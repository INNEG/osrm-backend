#ifndef OSRM_EXTRACTOR_GUIDANCE_SLIPROAD_HANDLER_HPP_
#define OSRM_EXTRACTOR_GUIDANCE_SLIPROAD_HANDLER_HPP_

#include "extractor/guidance/intersection.hpp"
#include "extractor/guidance/intersection_generator.hpp"
#include "extractor/guidance/intersection_handler.hpp"
#include "extractor/query_node.hpp"

#include "util/name_table.hpp"
#include "util/node_based_graph.hpp"

#include "util/geojson_debug_logger.hpp"
#include "util/geojson_debug_policies.hpp"

#include <cstddef>
#include <utility>
#include <vector>

#include <boost/optional.hpp>

namespace osrm
{
namespace extractor
{
namespace guidance
{

// Intersection handlers deal with all issues related to intersections.
// They assign appropriate turn operations to the TurnOperations.
class SliproadHandler final : public IntersectionHandler
{
  public:
    SliproadHandler(const IntersectionGenerator &intersection_generator,
                    const util::NodeBasedDynamicGraph &node_based_graph,
                    const std::vector<QueryNode> &node_info_list,
                    const util::NameTable &name_table,
                    const SuffixTable &street_name_suffix_table);

    ~SliproadHandler() override final = default;

    // check whether the handler can actually handle the intersection
    bool canProcess(const NodeID /*nid*/,
                    const EdgeID /*via_eid*/,
                    const Intersection & /*intersection*/) const override final;

    // process the intersection
    Intersection operator()(const NodeID nid,
                            const EdgeID via_eid,
                            Intersection intersection) const override final;

  private:
    util::ScopedGeojsonLoggerGuard<util::NodeIdVectorToLineString> geojson_lines;
    util::ScopedGeojsonLoggerGuard<util::NodeIdVectorToMultiPoint> geojson_points;

    struct IntersectionAndNode final
    {
        Intersection intersection;
        NodeID node;
    };

    // Returns a potential next intersection along a road skipping over traffic lights.
    boost::optional<IntersectionAndNode> getNextIntersection(const NodeID at,
                                                             const ConnectedRoad &road) const;

    boost::optional<std::size_t> getObviousIndexWithSliproads(const EdgeID from,
                                                              const Intersection &intersection,
                                                              const NodeID at) const;

    // Next intersection from `start` onto `onto` is too far away for a Siproad scenario
    bool nextIntersectionIsTooFarAway(const NodeID start, const EdgeID onto) const;

    // Through street: does a road continue with from's name at the intersection
    bool isThroughStreet(const EdgeID from, const Intersection &intersection) const;

    // Does the road from `current` to `next` continue
    bool roadContinues(const EdgeID current, const EdgeID next) const;

    // Is the area under the triangle a valid Sliproad triangle
    bool isValidSliproadArea(const NodeID, const NodeID, const NodeID) const;

    // Could a Sliproad reach this intersection?
    static bool canBeTargetOfSliproad(const Intersection &intersection);
};

} // namespace guidance
} // namespace extractor
} // namespace osrm

#endif /*OSRM_EXTRACTOR_GUIDANCE_SLIPROAD_HANDLER_HPP_*/
