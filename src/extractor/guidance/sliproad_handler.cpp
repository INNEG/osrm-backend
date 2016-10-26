#include "extractor/guidance/sliproad_handler.hpp"
#include "extractor/guidance/constants.hpp"
#include "extractor/guidance/toolkit.hpp"

#include "util/coordinate_calculation.hpp"
#include "util/guidance/toolkit.hpp"

#include <algorithm>
#include <iterator>
#include <limits>
#include <utility>

#include <boost/assert.hpp>

using EdgeData = osrm::util::NodeBasedDynamicGraph::EdgeData;
using osrm::util::guidance::getTurnDirection;
using osrm::util::guidance::angularDeviation;

namespace osrm
{
namespace extractor
{
namespace guidance
{

SliproadHandler::SliproadHandler(const IntersectionGenerator &intersection_generator,
                                 const util::NodeBasedDynamicGraph &node_based_graph,
                                 const std::vector<QueryNode> &node_info_list,
                                 const util::NameTable &name_table,
                                 const SuffixTable &street_name_suffix_table)
    : IntersectionHandler(node_based_graph,
                          node_info_list,
                          name_table,
                          street_name_suffix_table,
                          intersection_generator),
      geojson_lines{"sliproads-lines.geojson", node_info_list},
      geojson_points{"sliproads-points.geojson", node_info_list}
{
}

// The intersection has to connect a Sliproad, see the example scenario below:
// Intersection at `d`: Sliproad `bd` connecting `cd` and the road starting at `d`.
bool SliproadHandler::canProcess(const NodeID /*nid*/,
                                 const EdgeID /*via_eid*/,
                                 const Intersection &intersection) const
{
    return intersection.size() > 2;
}

// Detect sliproad b-d in the following example:
//
//       .
//       e
//       .
//       .
// a ... b .... c .
//       `      .
//         `    .
//           `  .
//              d
//              .
//
// ^ a nid
//    ^ ab source_edge_id
//       ^ b intersection
Intersection SliproadHandler::
operator()(const NodeID /*nid*/, const EdgeID source_edge_id, Intersection intersection) const
{
    BOOST_ASSERT(intersection.size() > 2);

    // Potential splitting / start of a Sliproad (b)
    auto intersection_node_id = node_based_graph.GetTarget(source_edge_id);

    // Road index prefering non-sliproads (bc)
    auto obvious = getObviousIndexWithSliproads(source_edge_id, intersection, intersection_node_id);

    if (!obvious)
        return intersection;

    // Potential non-sliproad road (bc), leading to the intersection (c) the Sliproad (bd) shortcuts
    const auto &next_road = intersection[*obvious];

    // The intersection `c` has to be reachable
    if (!next_road.entry_allowed)
    {
        return intersection;
    }

    // The road leading to the intersection (bc) has to continue from our source
    if (!roadContinues(source_edge_id, next_road.eid))
    {
        return intersection;
    }

    // Link-check for (bc) and later on (cd) which both are getting shortcutted by Sliproad
    const auto is_potential_link = [this, next_road](const ConnectedRoad &road) {
        // Prevent from starting in or going onto a roundabout
        auto is_roundabout = node_based_graph.GetEdgeData(road.eid).roundabout;
        auto onto_rounadbout = hasRoundaboutType(road.instruction);

        // Narrow turn angle for road (bd) and guard against data issues (overlapping roads)
        auto is_narrow = angularDeviation(road.angle, STRAIGHT_ANGLE) <= 2 * NARROW_TURN_ANGLE;
        auto not_same_angle =
            angularDeviation(next_road.angle, road.angle) > std::numeric_limits<double>::epsilon();

        auto roundabout = is_roundabout || onto_rounadbout;

        return !roundabout && road.entry_allowed && is_narrow && not_same_angle;
    };

    if (!std::any_of(begin(intersection), end(intersection), is_potential_link))
        return intersection;

    // If the intersection is too far away, don't bother continuing
    if (nextIntersectionIsTooFarAway(intersection_node_id, next_road.eid))
    {
        return intersection;
    }

    // Try to find the intersection at (c) which the Sliproad shortcuts
    const auto next = getNextIntersection(intersection_node_id, next_road);

    if (!next)
        return intersection;

    // If we are at a traffic loop at the end of a road, don't consider it a sliproad
    if (intersection_node_id == next->node)
        return intersection;

    std::vector<NameID> target_road_name_ids;
    target_road_name_ids.reserve(next->intersection.size());

    for (const auto &road : next->intersection)
    {
        const auto &target_data = node_based_graph.GetEdgeData(road.eid);
        target_road_name_ids.push_back(target_data.name_id);
    }

    // TODO: remove geojson debugging
    NodeID b, c, d;
    b = intersection_node_id;
    c = next->node;

    auto sliproad_found = false;

    // Check all roads for Sliproads and assign appropriate TurnType
    for (std::size_t road_index = 0, last = intersection.size(); road_index < last; ++road_index)
    {
        auto &sliproad = intersection[road_index]; // this is what we're checking and assigning to

        // This is what we know so far:
        //
        //       .
        //       e
        //       .
        //       .
        // a ... b .... c .   < `next` is intersection at `c`
        //       `      .
        //         `    .
        //           `  .
        //              d     < `target_intersection` is intersection at `d`
        //              .       `sliproad_edge_target` is node `d`
        //              e
        //
        //
        //          ^ `sliproad` is `bd`
        //       ^ `intersection` is intersection at `b`

        if (!is_potential_link(sliproad))
            continue;

        // b -> d edge id
        EdgeID sliproad_edge = sliproad.eid;

        // TODO: remove debugging
        d = node_based_graph.GetTarget(sliproad_edge);

        const auto target_intersection = [&](NodeID node) {
            auto intersection = intersection_generator(node, sliproad_edge);
            // skip over traffic lights
            if (intersection.size() == 2)
            {
                node = node_based_graph.GetTarget(sliproad_edge);
                sliproad_edge = intersection[1].eid;
                intersection = intersection_generator(node, sliproad_edge);
            }
            return intersection;
        }(intersection_node_id);

        const NodeID sliproad_edge_target = node_based_graph.GetTarget(sliproad_edge);

        // Distinct triangle nodes `bcd`
        if (intersection_node_id == next->node || intersection_node_id == sliproad_edge_target ||
            next->node == sliproad_edge_target)
        {
            continue;
        }

        // If the sliproad candidate is a through street, we cannot handle it as a sliproad.
        if (isThroughStreet(sliproad_edge, target_intersection))
        {
            continue;
        }

        // The turn off of the Sliproad has to be obvious and a narrow turn
        {
            const auto index = findObviousTurn(sliproad_edge, target_intersection);

            if (index == 0)
                continue;

            const auto onto = target_intersection[index];
            const auto angle_deviation = angularDeviation(onto.angle, STRAIGHT_ANGLE);
            const auto is_narrow_turn = angle_deviation <= 2 * NARROW_TURN_ANGLE;

            if (!is_narrow_turn)
                continue;
        }

        // Check for curvature. Depending on the turn's direction at `c`. Scenario for right turn:
        //
        // a ... b .... c .   a ... b .... c .   a ... b .... c .
        //       `      .           `  .   .             .    .
        //         `    .                . .           .      .
        //           `  .                 ..             .    .
        //              d                  d                . d
        //
        //                    Sliproad           Not a Sliproad
        {
            // Intersection is orderd: 0 is UTurn, then from sharp right to sharp left.
            // We already have an obvious index (bc) for going straight-ish.
            const auto is_right_turn = road_index < *obvious;
            const auto is_left_turn = road_index > *obvious;

            const auto &extractor = intersection_generator.GetCoordinateExtractor();

            const NodeID start = intersection_node_id; // b
            const EdgeID edge = sliproad_edge;         // bd

            const auto coords = extractor.GetForwardCoordinatesAlongRoad(start, edge);
            BOOST_ASSERT(coordinates.size() >= 2);

            // Now keep start and end coordinate fix and check for curvature
            const auto start_coord = coords.front();
            const auto end_coord = coords.back();

            const auto first = std::begin(coords) + 1;
            const auto last = std::end(coords) - 1;

            auto snuggles = false;

            using namespace util::coordinate_calculation;

            // In addition, if it's a right/left turn we expect the rightmost/leftmost
            // turn at `c` to be more or less ~90 degree for a Sliproad scenario.
            // We scale the 90 degrees to handle skewed small Sliproads scenarios.
            auto is_perpendicular_turn = false;

            const auto length = haversineDistance(node_info_list[intersection_node_id], //
                                                  node_info_list[next->node]);
            BOOST_ASSERT(length <= MAX_SLIPROAD_THRESHOLD);

            const double scale = length / MAX_SLIPROAD_THRESHOLD;
            BOOST_ASSERT(scale >= 0 && scale <= 1);

            // If we're at max distance we require a ~90 degree angle.
            // Below we linearly scale the required angle to account for small skewed scenarios.
            // TODO: 90+90+20 is probably too high
            double perpendicular_angle = 90 + (1.0 - scale) * 90 + FUZZY_ANGLE_DIFFERENCE;

            if (is_right_turn)
            {
                snuggles = std::all_of(first, last, [=](auto each) { //
                    return !isCCW(start_coord, each, end_coord);
                });

                const auto rightmost = next->intersection[1];
                is_perpendicular_turn = angularDeviation(rightmost.angle, STRAIGHT_ANGLE) <= //
                                        perpendicular_angle;                                 //
            }
            else if (is_left_turn)
            {
                snuggles = std::all_of(first, last, [=](auto each) { //
                    return isCCW(start_coord, each, end_coord);
                });

                const auto leftmost = next->intersection.back();
                is_perpendicular_turn = angularDeviation(leftmost.angle, STRAIGHT_ANGLE) <= //
                                        perpendicular_angle;                                //
            }

            if (!snuggles || !is_perpendicular_turn)
                continue;
        }

        // Check for area under triangle `bdc`.
        //
        // a ... b .... c .
        //       `      .
        //         `    .
        //           `  .
        //              d
        //
        if (!isValidSliproadArea(intersection_node_id, next->node, sliproad_edge_target))
        {
            continue;
        }

        // Check all roads at `d` if one is connected to `c`, is so `bd` is Sliproad.
        for (const auto &candidate_road : target_intersection)
        {
            const auto &candidate_data = node_based_graph.GetEdgeData(candidate_road.eid);

            // Name mismatch: check roads at `c` and `d` for same name
            const auto name_mismatch = [&](const NameID road_name_id) {
                return util::guidance::requiresNameAnnounced(road_name_id,              //
                                                             candidate_data.name_id,    //
                                                             name_table,                //
                                                             street_name_suffix_table); //
            };

            const auto candidate_road_name_mismatch = std::all_of(begin(target_road_name_ids), //
                                                                  end(target_road_name_ids),   //
                                                                  name_mismatch);              //

            if (candidate_road_name_mismatch)
                continue;

            if (node_based_graph.GetTarget(candidate_road.eid) == next->node)
            {
                sliproad.instruction.type = TurnType::Sliproad;
                sliproad_found = true;
                break;
            }
            else
            {
                const auto skip_traffic_light_intersection = intersection_generator(
                    node_based_graph.GetTarget(sliproad_edge), candidate_road.eid);
                if (skip_traffic_light_intersection.size() == 2 &&
                    node_based_graph.GetTarget(skip_traffic_light_intersection[1].eid) ==
                        next->node)
                {

                    sliproad.instruction.type = TurnType::Sliproad;
                    sliproad_found = true;
                    break;
                }
            }
        }
    }

    // Now in case we found a Sliproad and assigned the corresponding type to the road,
    // it could be that the intersection from which the Sliproad splits off was a Fork before.
    // In those cases the obvious non-Sliproad is now obvious and we discard the Fork turn type.
    if (sliproad_found && next_road.instruction.type == TurnType::Fork)
    {
        const auto &source_edge_data = node_based_graph.GetEdgeData(source_edge_id);
        const auto &next_data = node_based_graph.GetEdgeData(next_road.eid);

        const auto same_name = source_edge_data.name_id != EMPTY_NAMEID && //
                               next_data.name_id != EMPTY_NAMEID &&        //
                               !util::guidance::requiresNameAnnounced(source_edge_data.name_id,
                                                                      next_data.name_id,
                                                                      name_table,
                                                                      street_name_suffix_table); //

        if (same_name)
        {
            if (angularDeviation(next_road.angle, STRAIGHT_ANGLE) < 5)
                intersection[*obvious].instruction.type = TurnType::Suppressed;
            else
                intersection[*obvious].instruction.type = TurnType::Continue;
            intersection[*obvious].instruction.direction_modifier =
                getTurnDirection(intersection[*obvious].angle);
        }
        else if (next_data.name_id != EMPTY_NAMEID)
        {
            intersection[*obvious].instruction.type = TurnType::NewName;
            intersection[*obvious].instruction.direction_modifier =
                getTurnDirection(intersection[*obvious].angle);
        }
        else
        {
            intersection[*obvious].instruction.type = TurnType::Suppressed;
        }
    }

    if (sliproad_found)
    {
        geojson_points.Write(std::vector<NodeID>{b, c, d});
        geojson_lines.Write(std::vector<NodeID>{b, d});
    }

    return intersection;
}

// Implementation details

// Skips over `tl` traffic light and returns
//
//  a ... tl ... b .. c
//               .
//               .
//               d
//
//  ^ at
//     ^ road
boost::optional<SliproadHandler::IntersectionAndNode>
SliproadHandler::getNextIntersection(const NodeID at, const ConnectedRoad &road) const
{
    auto intersection = intersection_generator(at, road.eid);
    auto in_edge = road.eid;
    auto intersection_node = node_based_graph.GetTarget(in_edge);

    // To prevent ending up in an endless loop, we remember all visited nodes. This is
    // necessary, since merging of roads can actually create enterable loops of degree two
    std::unordered_set<NodeID> visited_nodes;

    auto node = at;
    while (intersection.size() == 2 && visited_nodes.count(node) == 0)
    {
        visited_nodes.insert(node);
        node = node_based_graph.GetTarget(in_edge);

        // We ended up in a loop without exit
        if (node == at)
        {
            return boost::none;
        }

        in_edge = intersection[1].eid;
        intersection = intersection_generator(node, in_edge);
        intersection_node = node_based_graph.GetTarget(in_edge);
    }

    if (intersection.size() <= 2)
    {
        return boost::none;
    }

    return boost::make_optional(IntersectionAndNode{intersection, intersection_node});
}

boost::optional<std::size_t> SliproadHandler::getObviousIndexWithSliproads(
    const EdgeID from, const Intersection &intersection, const NodeID at) const
{
    BOOST_ASSERT(from != SPECIAL_EDGEID);
    BOOST_ASSERT(at != SPECIAL_NODEID);

    // If a turn is obvious without taking Sliproads into account use this
    const auto index = findObviousTurn(from, intersection);

    if (index != 0)
        return boost::make_optional(index);

    // Otherwise check if the road is forking into two and one of them is a Sliproad;
    // then the non-Sliproad is the obvious one.
    if (intersection.size() != 3)
        return boost::none;

    const auto forking = intersection[1].instruction.type == TurnType::Fork &&
                         intersection[2].instruction.type == TurnType::Fork;

    if (!forking)
        return boost::none;

    const auto first = getNextIntersection(at, intersection[1]);
    const auto second = getNextIntersection(at, intersection[2]);

    if (!first || !second)
        return boost::none;

    // In case of loops at the end of the road, we will arrive back at the intersection
    // itself. If that is the case, the road is obviously not a sliproad.
    if (canBeTargetOfSliproad(first->intersection) && at != second->node)
        return boost::make_optional(std::size_t{2});

    if (canBeTargetOfSliproad(second->intersection) && at != first->node)
        return boost::make_optional(std::size_t{1});

    return boost::none;
}

bool SliproadHandler::nextIntersectionIsTooFarAway(const NodeID start, const EdgeID onto) const
{
    BOOST_ASSERT(start != SPECIAL_NODEID);
    BOOST_ASSERT(onto != SPECIAL_EDGEID);

    const auto &extractor = intersection_generator.GetCoordinateExtractor();

    // TODO: refactor, could be useful for other scenarios, too

    struct NextIntersectionDistanceAccumulator
    {
        NextIntersectionDistanceAccumulator(
            const extractor::guidance::CoordinateExtractor &extractor_,
            const util::NodeBasedDynamicGraph &graph_)
            : extractor{extractor_}, graph{graph_}, too_far_away{false}, distance{0}
        {
        }

        bool terminate()
        {
            if (distance > MAX_SLIPROAD_THRESHOLD)
            {
                too_far_away = true;
                return true;
            }

            return false;
        }

        void update(const NodeID start, const EdgeID onto, const NodeID)
        {
            using namespace util::coordinate_calculation;

            const auto coords = extractor.GetForwardCoordinatesAlongRoad(start, onto);
            distance += getLength(coords, &haversineDistance);
        }

        const extractor::guidance::CoordinateExtractor &extractor;
        const util::NodeBasedDynamicGraph &graph;
        bool too_far_away;
        double distance;
    } accumulator{extractor, node_based_graph};

    struct /*TrafficSignalBarrierRoadSelector*/
    {
        boost::optional<EdgeID> operator()(const NodeID,
                                           const EdgeID,
                                           const Intersection &intersection,
                                           const util::NodeBasedDynamicGraph &) const
        {
            if (intersection.size() == 2)
                return boost::make_optional(intersection[1].eid);
            else
                return boost::none;
        }
    } const selector;

    (void)graph_walker.TraverseRoad(start, onto, accumulator, selector);

    return accumulator.too_far_away;
}

bool SliproadHandler::isThroughStreet(const EdgeID from, const Intersection &intersection) const
{
    BOOST_ASSERT(from != SPECIAL_EDGEID);
    BOOST_ASSERT(!intersection.empty());

    const auto edge_name_id = node_based_graph.GetEdgeData(from).name_id;

    auto first = begin(intersection) + 1; // Skip UTurn road
    auto last = end(intersection);

    auto same_name = [&](const auto &road) {
        const auto road_name_id = node_based_graph.GetEdgeData(road.eid).name_id;

        return edge_name_id != EMPTY_NAMEID && //
               road_name_id != EMPTY_NAMEID && //
               !util::guidance::requiresNameAnnounced(edge_name_id,
                                                      road_name_id,
                                                      name_table,
                                                      street_name_suffix_table); //
    };

    return std::find_if(first, last, same_name) != last;
}

bool SliproadHandler::roadContinues(const EdgeID current, const EdgeID next) const
{
    const auto current_data = node_based_graph.GetEdgeData(current);
    const auto next_data = node_based_graph.GetEdgeData(next);

    auto same_road_category = current_data.road_classification == next_data.road_classification;
    auto same_travel_mode = current_data.travel_mode == next_data.travel_mode;

    auto same_name = current_data.name_id != EMPTY_NAMEID && //
                     next_data.name_id != EMPTY_NAMEID &&    //
                     !util::guidance::requiresNameAnnounced(current_data.name_id,
                                                            next_data.name_id,
                                                            name_table,
                                                            street_name_suffix_table); //

    const auto continues = same_road_category && same_travel_mode && same_name;
    return continues;
}

bool SliproadHandler::isValidSliproadArea(const NodeID a, const NodeID b, const NodeID c) const

{
    using namespace util::coordinate_calculation;

    const auto first = node_info_list[a];
    const auto second = node_info_list[b];
    const auto third = node_info_list[c];

    const auto length = haversineDistance(first, second);
    const auto heigth = haversineDistance(second, third);

    const auto area = (length * heigth) / 2.;

    // Everything below is data issue - there are some weird situations where
    // nodes are really close to each other and / or tagging ist just plain off.
    const constexpr auto MIN_SLIPROAD_AREA = 3.;

    if (area < MIN_SLIPROAD_AREA || area > MAX_SLIPROAD_THRESHOLD * MAX_SLIPROAD_THRESHOLD)
        return false;

    return true;
}

bool SliproadHandler::canBeTargetOfSliproad(const Intersection &intersection)
{
    // Example to handle:
    //       .
    // a . . b .
    //  `    .
    //    `  .
    //       c    < intersection
    //       .
    //

    // One outgoing two incoming
    if (intersection.size() != 3)
        return false;

    // For (c) to be target of a Sliproad (ab) and (bc) have to be directed
    auto backwards = intersection[0].entry_allowed;
    auto undirected_link = intersection[1].entry_allowed && intersection[2].entry_allowed;

    if (backwards || undirected_link)
        return false;

    return true;
}

} // namespace guidance
} // namespace extractor
} // namespace osrm
