/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "Supergraph.hpp"

#include <rmf_utils/math.hpp>

#include <unordered_set>

#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__SUPERGRAPH
#include <iostream>
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__SUPERGRAPH

namespace rmf_traffic {
namespace agv {
namespace planning {

namespace {
//==============================================================================
Supergraph::FloorChangeMap find_floor_changes( //找到并标识出哪些lane存在楼层切换，将这些变化的标试点放到一个队列中
    const Graph::Implementation& original)
{
  Supergraph::FloorChangeMap all_floor_changes;

  for (std::size_t i = 0; i < original.waypoints.size(); ++i)
  {
    const auto& initial_map_name = original.waypoints[i].get_map_name();
    auto& floor_changes = all_floor_changes[initial_map_name];

    for (const auto l : original.lanes_from[i])
    {
      const auto& lane = original.lanes[l];
      const auto& exit = original.waypoints[lane.exit().waypoint_index()];
      const auto& final_map_name = exit.get_map_name();
      if (initial_map_name != final_map_name)
        floor_changes[final_map_name].push_back(Supergraph::FloorChange{l});
    }
  }

  return all_floor_changes;
}

//==============================================================================
Eigen::Rotation2Dd compute_forward_offset(
    const Eigen::Vector2d& forward)
{
  return Eigen::Rotation2Dd(std::atan2(forward[1], forward[0]));
}

//==============================================================================
struct TraversalNode
{
  std::size_t initial_lane_index;
  std::size_t finish_lane_index;
  std::size_t finish_waypoint_index;

  Eigen::Vector2d initial_p;
  Eigen::Vector2d finish_p;

  Graph::Lane::EventPtr entry_event;
  Graph::Lane::EventPtr exit_event;

  // TODO(MXG): Can std::string_view be used to make this more memory efficient?
  std::vector<std::string> map_names;

  std::array<std::optional<double>, 2> orientations;
  bool standstill = false;
};

//==============================================================================
bool valid_traversal(const TraversalNode& node)
{
  if (node.standstill)
    return true;

  for (const auto orientation : node.orientations)
  {
    if (orientation.has_value())
      return true;
  }

  return false;
}

//==============================================================================
void node_to_traversals( //将node转换成traversals
  const TraversalNode& node,
  const TraversalGenerator::Kinematics& kin,
  std::vector<Traversal>& output)
{

  assert(valid_traversal(node));
  Traversal traversal;
  traversal.initial_lane_index = node.initial_lane_index;
  traversal.finish_lane_index = node.finish_lane_index;
  traversal.finish_waypoint_index = node.finish_waypoint_index;
  traversal.best_time = 0.0;
  traversal.maps = std::vector<std::string>(
        node.map_names.begin(), node.map_names.end());

//  std::cout << "Traversal [" << traversal.initial_lane_index << "] -> ("
//            << traversal.finish_lane_index << "): entry event {"
//            << traversal.entry_event << "}" << std::endl;
  if (node.entry_event)
  {
    traversal.entry_event = node.entry_event->clone();
    traversal.best_time += rmf_traffic::time::to_seconds(
          traversal.entry_event->duration());
  }

  if (node.exit_event)
  {
    traversal.exit_event = node.exit_event->clone();
    traversal.best_time += rmf_traffic::time::to_seconds(
          traversal.exit_event->duration());
  }

  if (node.standstill)
  {
    Traversal::Alternative alt;
    alt.routes = make_start_factory(
      node.initial_p, std::nullopt, kin.limits,
      kin.interpolate.rotation_thresh, traversal.maps);

    // If the node is a standstill, just add this empty Alternative
    traversal.alternatives[static_cast<std::size_t>(Orientation::Any)]
        = std::move(alt);

    // If the node is a standstill, it should't have any orientation
    // requirements
    assert(
      !node.orientations[static_cast<std::size_t>(Orientation::Forward)]
      .has_value() &&
      !node.orientations[static_cast<std::size_t>(Orientation::Backward)]
      .has_value()
    );
  }

  std::optional<double> best_trajectory_time;
  for (std::size_t i=0; i < 2; ++i)
  {
    const auto yaw = node.orientations[i];

    if (!yaw.has_value())
      continue;

    const Eigen::Vector3d start{
      node.initial_p.x(),
      node.initial_p.y(),
      *yaw
    };

    const Eigen::Vector3d finish{
      node.finish_p.x(),
      node.finish_p.y(),
      *yaw
    };

    Traversal::Alternative alternative;
    alternative.yaw = *yaw;
    auto factory_info = make_differential_drive_translate_factory(
          start, finish, kin.limits,
          kin.interpolate.translation_thresh,
          kin.interpolate.rotation_thresh,
          traversal.maps);

    const auto time = factory_info.minimum_cost; //估算的robot在直线上匀加速匀速匀减速的时间
    alternative.time = time;
    alternative.routes = std::move(factory_info.factory); // 注意，它是一个函数， 根据给定参数生成路径
    // routes 是考虑了robot转向的时间， 它生成的路径包括robot原地旋转，直线运动
#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__SUPERGRAPH
    std::cout << "SUPERGRAPH [" << traversal.initial_lane_index
              << Orientation(i) << Side::Start << "] ("
              << traversal.finish_waypoint_index << "): " << time << std::endl;
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__SUPERGRAPH

    if (!best_trajectory_time.has_value() || time < *best_trajectory_time)
      best_trajectory_time = time;

    traversal.alternatives[i] = std::move(alternative);
  }

  if (best_trajectory_time.has_value())
    traversal.best_time += *best_trajectory_time;

  output.emplace_back(std::move(traversal));
}

//==============================================================================
void add_if_missing(
    std::vector<std::string>& all_maps,
    const std::string& map)
{
  if (std::find(all_maps.begin(), all_maps.end(), map) == all_maps.end())
    all_maps.push_back(map);
}

//==============================================================================
void perform_traversal( // 模拟robot从一个点移动到另一个点，并计算它的轨迹，时间信息
  const TraversalNode* parent, // top
  const std::size_t lane_index, // l
  const Graph::Implementation& graph,
  const TraversalGenerator::Kinematics& kin,
  std::vector<TraversalNode>& queue,
  std::vector<Traversal>& output,
  std::unordered_set<std::size_t>& visited)
{
  const auto& lane = graph.lanes[lane_index];
  const auto& entry = lane.entry();
  const auto& exit = lane.exit();
  const std::size_t wp_index_0 = entry.waypoint_index(); //robot目前在entry点
  const std::size_t wp_index_1 = exit.waypoint_index();

  if (!visited.insert(wp_index_1).second) //试图将exit加入visited表格中
  {
    // If we have already added the finish waypoint to the queue, then there is
    // no need to add it again.
    return;
  }

  const auto& wp0 = graph.waypoints[wp_index_0];
  const auto& wp1 = graph.waypoints[wp_index_1];
  const Eigen::Vector2d p0 = wp0.get_location();
  const Eigen::Vector2d p1 = wp1.get_location();

  TraversalNode node;
  node.finish_lane_index = lane_index;
  node.finish_waypoint_index = wp_index_1;

  if (parent) // 是否有父节点
  {
    if (entry.event())
    {
      // If this lane has an entry event, then we cannot continue the traversal.
      // A new traversal will have to begin from this waypoint.
      return;
    }

    node.initial_lane_index = parent->initial_lane_index; //如果有，则lane的初始点，也就是entry点就是父节点的初始点
    node.initial_p = parent->initial_p; // 这里为啥不是parent->finish_p， 
    node.map_names = parent->map_names;

    if (parent->entry_event)
      node.entry_event = parent->entry_event->clone();
  }
  else
  {
    node.initial_lane_index = lane_index; //如果是初始点，则lane的初始index和结束index都是当前lane的ID
    node.initial_p = p0;

    if (const auto* entry_event = entry.event())
      node.entry_event = entry_event->clone();
  }

  node.finish_p = p1;//lane的exit点

  add_if_missing(node.map_names, wp0.get_map_name());
  add_if_missing(node.map_names, wp1.get_map_name());

  const double dist = (p1 - p0).norm(); //lane的长度
  if (!kin.constraint.has_value() || dist < kin.interpolate.translation_thresh)
  {
    if (!parent)
    {
      // These waypoints are effectively on top of each other, and we haven't
      // moved anywhere to arrive here. We will call this a standstill.
      node.standstill = true;
    }
    else
    {
      // These waypoints are effectively on top of each other. We will carry
      // over the orientations of the parent.
      node.standstill = parent->standstill;
      node.orientations = parent->orientations;
    }
  }
  else
  {
    const Eigen::Vector2d course_vector = (p1 - p0)/dist;
    const auto orientations =
        kin.constraint->get_orientations(course_vector); // 拿到robot与lane的有向角

    const double thresh = kin.interpolate.rotation_thresh;

    for (std::size_t i = 0; i < orientations.size(); ++i)
    {
      const auto orientation = orientations[i];
      if (!orientation.has_value())
        continue;

      const auto* entry_constraint = entry.orientation_constraint();
      if (!orientation_constraint_satisfied( //robot 能否旋转到lane接受的入口角？ 如果可以，则继续，不行则跳过该点
            p0, *orientation, course_vector, entry_constraint, thresh))
        continue;

      const auto* exit_constraint = exit.orientation_constraint();
      if (!orientation_constraint_satisfied(
            p1, *orientation, course_vector, exit_constraint, thresh))
        continue;

      if (parent && !parent->standstill) // standstill应该是指独立存在的一个点或者说所连接的lane长度很小
      {
        const auto parent_orientation = parent->orientations[i];
        if (!parent_orientation.has_value())
          continue;

        const double R_diff = rmf_utils::wrap_to_pi(
              *orientation - *parent_orientation);

        if (std::abs(R_diff) > kin.interpolate.rotation_thresh)
          continue;
      }

      node.orientations[i] = *orientation;
    }
  }

  if (!valid_traversal(node))
  {
    // If this lane has no valid orientations and also does not stand still,
    // then it's not a real traversal, and we should not output it or queue it.
    return;
  }

  const auto* exit_event = exit.event();
  if (exit_event)
    node.exit_event = exit_event->clone();

  // Convert this node into a set of traversals
  node_to_traversals(node, kin, output);

  if (exit_event)
  {
    // If this lane has an exit event, then we need to stop the traversal here,
    // so it does not get added to the queue.
    return;
  }

//  std::cout << "Pushing for further expansion" << std::endl;
  queue.push_back(std::move(node));
}

//==============================================================================
void expand_traversal(
  const TraversalNode& parent,
  const std::size_t lane_index,
  const Graph::Implementation& graph,
  const TraversalGenerator::Kinematics& kin,
  std::vector<TraversalNode>& queue,
  std::vector<Traversal>& output,
  std::unordered_set<std::size_t>& visited)
{
  perform_traversal(&parent, lane_index, graph, kin, queue, output, visited);
}

//==============================================================================
void initiate_traversal(
  const std::size_t lane_index,
  const Graph::Implementation& graph,
  const TraversalGenerator::Kinematics& kin,
  std::vector<TraversalNode>& queue,
  std::vector<Traversal>& output,
  std::unordered_set<std::size_t>& visited)
{
  perform_traversal(nullptr, lane_index, graph, kin, queue, output, visited);
}

} // anonymous namespace

//==============================================================================
bool orientation_constraint_satisfied(
    const Eigen::Vector2d p,
    const double orientation,
    const Eigen::Vector2d course_vector,
    const rmf_traffic::agv::Graph::OrientationConstraint* constraint,
    const double rotation_thresh)
{
  if (!constraint)
    return true;

  Eigen::Vector3d position{p.x(), p.y(), orientation};
  constraint->apply(position, course_vector); // 找到与constraint可接受的最小有向角的入口方向， 并对目前的waypoint的方向进行修正
  const double diff = rmf_utils::wrap_to_pi(position[2] - orientation); // 这个差异既是robot需要旋转的角度
  if (std::abs(diff) > rotation_thresh) // 超出了robot允许的旋转角度， 对于differential wheeled robot, this limitation should not be a issue, as it can rotate at a same point
    return false;

  return true;
}

//==============================================================================
const Eigen::Rotation2Dd DifferentialDriveConstraint::R_pi =
  Eigen::Rotation2Dd(M_PI);

//==============================================================================
DifferentialDriveConstraint::DifferentialDriveConstraint(
    const Eigen::Vector2d& forward,
    const bool reversible)
: R_f_inv(compute_forward_offset(forward).inverse()), //这里的inverse很重要，
  reversible(reversible)
{
  // Do nothing
}

//==============================================================================
std::array<std::optional<double>, 2>
DifferentialDriveConstraint::get_orientations( // 其实就是计算robot的朝向到给定的方向间的夹角，但是是带有方向的，以顺时针为正
    const Eigen::Vector2d& course_vector) const
{
  std::array<std::optional<double>, 2> orientations;

  const Eigen::Rotation2Dd R_c(
    std::atan2(course_vector[1], course_vector[0]));
  const Eigen::Rotation2Dd R_h = R_c * R_f_inv; // * 运算被重置成两个角度相加，Rotation2Dd的角度是按顺时针方向为正， R_f_inv是角度取负操作

  orientations[static_cast<std::size_t>(Orientation::Forward)]
      = rmf_utils::wrap_to_pi(R_h.angle());

  if (reversible)
  {
    orientations[static_cast<std::size_t>(Orientation::Backward)]
        = rmf_utils::wrap_to_pi((R_pi * R_h).angle());
  }

  return orientations;
}

//==============================================================================
TraversalGenerator::Kinematics::Kinematics(
  const VehicleTraits& traits,
  const Interpolate::Options::Implementation& interpolate_)
: limits(VehicleTraits::Implementation::get_limits(traits)),
  interpolate(interpolate_)
{
  if (const auto* diff_drive = traits.get_differential())
  {
    constraint = DifferentialDriveConstraint(
          diff_drive->get_forward(), diff_drive->is_reversible());
  }
}

//==============================================================================
TraversalGenerator::TraversalGenerator(
  const std::shared_ptr<const Supergraph>& graph)
: _graph(graph),
  _kinematics(graph->traits(), graph->options())
{
  // Do nothing
}

//==============================================================================
ConstTraversalsPtr TraversalGenerator::generate(
    const std::size_t& key,
    const Storage&, // old items are irrelevant
    Storage& new_items) const
{
  const auto supergraph = _graph.lock();

  // TODO(MXG): When we have C++20 support, we can label this with [[unlikely]]
  if (!supergraph)
  {
    // This means the supergraph that's being traversed has destructed while
    // this cache is still alive. That's really weird and shouldn't happen.
    // The only reason we keep the graph as a nullptr is
    // 1) to avoid a circular dependency
    // 2) we cannot technically guarantee that the cache's lifecycle will fit
    //    within the supergraph's lifecycle, and throwing an exception is
    //    preferable to Undefined Behavior.
    throw std::runtime_error(
          "[rmf_traffic::agv::planning::TraversalGenerator::generate] "
          "Supergraph died while a TraversalCache was still being used. "
          "Please report this critical bug to the maintainers of rmf_traffic.");
  }

  const std::size_t waypoint_index = key;
  const auto& graph = supergraph->original();

  // 起点在waypoint的lanes
  const auto& initial_lanes = graph.lanes_from[waypoint_index]; // lane的起点
  std::vector<TraversalNode> queue;
  std::vector<Traversal> output;
  std::unordered_set<std::size_t> visited;
  visited.insert(waypoint_index);

  for (const auto l : initial_lanes)
    // 主要是初始化robot走到每条lane的出口点的时间，及生成对应的tajectory
    // output 记录的是模拟robot移动的轨迹信息， queue 是output的缩略信息， visited记录的是waypoint是否被访问
    initiate_traversal(l, graph, _kinematics, queue, output, visited); // _kinematics 是robot的motion dynamic， 在初始化TraversalGenerator时进行初始化

  while (!queue.empty())
  {
    auto top = std::move(queue.back());
    queue.pop_back();

    const auto& lanes = graph.lanes_from[top.finish_waypoint_index];
    for (const auto l : lanes)
      expand_traversal(top, l, graph, _kinematics, queue, output, visited);
  }

  // using Traversals = std::vector<Traversal>; //Traversal可以理解为一条轨迹
  auto new_traversals = std::make_shared<Traversals>(std::move(output));  // output记录了所有拓展的lane和waypoint，及计算trajectory的function
  new_items.insert({waypoint_index, new_traversals});

  return new_traversals;
}

//==============================================================================
std::shared_ptr<const Supergraph> Supergraph::make(
    Graph::Implementation original,
    VehicleTraits traits,
    const Interpolate::Options::Implementation& interpolate)
{
  auto supergraph = std::shared_ptr<Supergraph>(
        new Supergraph(std::move(original), std::move(traits), interpolate));

  supergraph->_traversals =
      CacheManager<TraversalCache>::make(
        std::make_shared<TraversalGenerator>(supergraph));

  supergraph->_entries_into_waypoint_cache = // 这是唯一的初始化，其它地方没有再进行任何赋值， 它只是一个指针
      CacheManager<EntriesCache>::make(
        std::make_shared<EntriesGenerator>(supergraph));

  const std::size_t N_lanes = supergraph->original().lanes.size();
  supergraph->_lane_yaw_cache =
      CacheManager<LaneYawCache>::make(
        std::make_shared<LaneYawGenerator>(supergraph),
        [N_lanes](){ return LaneYawMap(251, EntryHash(N_lanes)); }); //使用新的够着函数

  return supergraph;
}

//==============================================================================
const Graph::Implementation& Supergraph::original() const
{
  return _original;
}

//==============================================================================
const VehicleTraits& Supergraph::traits() const
{
  return _traits;
}

//==============================================================================
const Interpolate::Options::Implementation& Supergraph::options() const
{
  return _interpolate;
}

//==============================================================================
auto Supergraph::floor_change() const -> const FloorChangeMap&
{
  return _floor_changes;
}

//==============================================================================
ConstTraversalsPtr Supergraph::traversals_from(
    const std::size_t waypoint_index) const
{
  // 调用 TraversalGenerator::generate 构造ConstTraversalsPtr
  // CacheManager<TraversalCache> _traversals;
  // _traversals->get() -> TraversalCache
  // get(waypoint_index) -> unordered_map<std::size_t, ConstTraversalsPtr> -> ConstTraversalsPtr
  return _traversals->get().get(waypoint_index);
}

//==============================================================================
std::vector<Supergraph::Entry> Supergraph::Entries::relevant_entries(
    std::optional<double> orientation_opt) const
{
  std::vector<Supergraph::Entry> output;
  output.reserve(_total_entries);
  if (_agnostic_entry.has_value()) 
  {
    output.push_back(*_agnostic_entry); // 不需要考虑robot的转向
  }

  if (!orientation_opt.has_value())
  {
    // If there isn't a specific orientation being asked for, then we need to
    // consider every possible entry.
    for (const auto& [_, entry] : _angled_entries)
      output.push_back(entry);

    return output;
  }

  if (_angled_entries.empty())
    return output;

  const double orientation = rmf_utils::wrap_to_pi(*orientation_opt);
  const double lower_bound = _angled_entries.begin()->first; // 正常迭代，从map头开始
  const double upper_bound = _angled_entries.rbegin()->first; // 返回一个反向迭代器，指向map的末尾并向map的头部迭代
  if (orientation < lower_bound || upper_bound < orientation) // 说明新的数据不在_angled_entries中， 则将map的头和尾放到output中返回
  {
    output.push_back(_angled_entries.begin()->second);
    if (lower_bound != upper_bound)
      output.push_back(_angled_entries.rbegin()->second);

    return output;
  }

  const auto it = _angled_entries.lower_bound(orientation); // 找到第一个大于或等于orientation的位置。
  output.push_back(it->second);
  if (orientation < it->first) //应该是不能发生吧？？？
  {
    // it cannot be begin() because the earlier if-statement would have caught
    // it if it were. So we can safely decrement and dereference this iterator,
    // and it will certainly provide the lower bound for the requested
    // orientation.
    output.push_back((--std::map<double, Entry>::const_iterator(it))->second);
  }

  return output;
}

//==============================================================================
Supergraph::Entries::Entries(
  std::map<double, Entry> angled_entries,
  std::optional<Entry> agnostic_entry)
: _angled_entries(std::move(angled_entries)),
  _agnostic_entry(std::move(agnostic_entry))
{
  _total_entries = _angled_entries.size()
      + (_agnostic_entry.has_value()? 1 : 0);
}

//==============================================================================
Supergraph::ConstEntriesPtr Supergraph::entries_into(
  const std::size_t waypoint_index) const
{
  // 主要是趣调用 Supergraph::EntriesGenerator::generate 函数，该函数会添加waypoint_index到storage中
  return _entries_into_waypoint_cache->get().get(waypoint_index);
}

//==============================================================================
std::optional<double> Supergraph::yaw_of(const Entry& entry) const
{
  // TODO(MXG): Try going back to the caching system when time permits.
//  if (entry.orientation == Orientation::Any)
//    return std::nullopt;

//  return _lane_yaw_cache->get().get(entry);


  if (entry.orientation == Orientation::Any)
    return std::nullopt;

  if (!_constraint.has_value())
    return std::nullopt;

  const auto& lane = _original.lanes[entry.lane];
  const std::size_t waypoint_index_0 = lane.entry().waypoint_index();
  const std::size_t waypoint_index_1 = lane.exit().waypoint_index();
  const auto& wp0 = _original.waypoints[waypoint_index_0];
  const auto& wp1 = _original.waypoints[waypoint_index_1];

  const Eigen::Vector2d p0 = wp0.get_location();
  const Eigen::Vector2d p1 = wp1.get_location();
  const double dist = (p1 - p0).norm(); // lane的长度
  if (dist <= _interpolate.translation_thresh)
    return std::nullopt;

  const Eigen::Vector2d course_vector = (p1 - p0)/dist;
  const auto orientations = _constraint->get_orientations(course_vector);//robot与lane的有向夹角
  return orientations[static_cast<std::size_t>(entry.orientation)];
}

//==============================================================================
DifferentialDriveKeySet Supergraph::keys_for(
  const std::size_t start_waypoint_index,
  const std::size_t goal_waypoint_index,
  std::optional<double> goal_orientation) const
{
  using KeyHash = DifferentialDriveMapTypes::KeyHash;
  DifferentialDriveKeySet keys(31, KeyHash{_original.lanes.size()}); // 31为set的大小， KeyHash定义了hash函数
  // unordered_set的构造函数为， unordered_set(size_type bucket_count,
  // const Hash& hash = Hash(),
  // const key_equal& equal = key_equal(),
  // const Allocator& alloc = Allocator() );)

  // 找到与goal_orientation方向最接近的lane作为robot的最终入口点
  const auto relevant_entries = entries_into(goal_waypoint_index) // entries_into是去构建给定waypoint点，比如goal_waypoint_index，的所有出口在该点的lanes，并计算它们与robot之间的有向角，这为计算huristic值提供便利
      ->relevant_entries(goal_orientation); // Entries的relevant_entries函数是在entries_into函数找到的所有lane中与goal_orientation方向最接近的lane

  const auto relevant_traversals = traversals_from(start_waypoint_index);
  assert(relevant_traversals);

  for (const auto& traversal : *relevant_traversals) // 会导致多个重复的key？？？？
  {
    const std::size_t lane_index = traversal.initial_lane_index;
    for (std::size_t orientation = 0; orientation < 3; ++orientation)
    {
      const auto& alt = traversal.alternatives[orientation];
      if (!alt.has_value())
        continue;

      for (const auto& entry : relevant_entries)
      {
        keys.insert(
          {
            lane_index, Orientation(orientation), Side::Start,
            entry.lane, entry.orientation
          });
      }
    }
  }

  return keys;
}

//==============================================================================
Supergraph::EntriesGenerator::EntriesGenerator(
  const std::shared_ptr<const Supergraph>& graph)
: _graph(graph)
{
  if (const auto* differential = graph->traits().get_differential())
  {
    _constraint = DifferentialDriveConstraint(
          differential->get_forward(),
          differential->is_reversible());
  }
}

//==============================================================================
auto Supergraph::EntriesGenerator::generate( // 通向key所在的waypoint的所有lane与robot朝向的有向角
  const std::size_t& key,
  const Storage&, // old items are irrelevant
  Storage& new_items) const -> ConstEntriesPtr
{
  const auto supergraph = _graph.lock();

  // TODO(MXG): When we have C++20 support, we can label this with [[unlikely]]
  if (!supergraph)
  {
    // This means the supergraph that's being traversed has destructed while
    // this cache is still alive. That's really weird and shouldn't happen.
    // The only reason we keep the graph as a nullptr is
    // 1) to avoid a circular dependency
    // 2) we cannot technically guarantee that the cache's lifecycle will fit
    //    within the supergraph's lifecycle, and throwing an exception is
    //    preferable to Undefined Behavior.
    throw std::runtime_error(
          "[rmf_traffic::agv::planning::Supergraph::EntriesGenerator::generate]"
          " Supergraph died while a EntriesCache was still being used. "
          "Please report this critical bug to the maintainers of rmf_traffic.");
  }

  const std::size_t waypoint_index = key;
  const auto& graph = supergraph->original();
  const auto& interpolate = supergraph->options();
  const auto& entry_lanes = graph.lanes_into[waypoint_index]; //找出所有终点为该waypoint点的lane， 也即lane的exit是该点的lane

  std::map<double, Entry> angled_entries;
  std::optional<Entry> agnostic_entry;

  const Eigen::Vector2d p1 = graph.waypoints[waypoint_index].get_location();

  for (const auto& lane_index : entry_lanes) //所有的lane出口方向与robot所形成的有向角， 它可以用于计算robot到达该waypoint后原地旋转的成本
  {
    const auto& lane = graph.lanes[lane_index];
    const auto& wp0 = graph.waypoints[lane.entry().waypoint_index()];
    const Eigen::Vector2d p0 = wp0.get_location();

    const double dist = (p1 - p0).norm(); //计算的是lane的长度
    if (!_constraint.has_value() || dist < interpolate.translation_thresh)
    {
      agnostic_entry = Entry{lane_index, Orientation::Any, Side::Finish}; // 如果该lane太短，则没必要考虑入场角度， 因为可以认为它已经到达该点，或者两点可以认为是同一点？？？ 无约束，比如omnidirection的robot，则不需要考虑入场角度
    }
    else
    {
      const Eigen::Vector2d course_vector = (p1 - p0)/dist; // lane的方向
      const auto orientations = // 计算robot当前方向到lane的方向所需要旋转的角度，顺时针为正
          _constraint->get_orientations(course_vector);

      for (std::size_t i=0; i < orientations.size(); ++i)
      {
        const auto orientation = orientations[i];
        if (!orientation.has_value())
          continue;

        // Orientation 指定robot的操作方向，向前还是向后
        angled_entries.insert(
          {*orientation, Entry{lane_index, Orientation(i), Side::Finish}});
      }
    }
  }

  auto new_entries = std::make_shared<Entries>( //指向Entries的指针
        std::move(angled_entries), std::move(agnostic_entry));

  new_items.insert({key, new_entries});
  return new_entries;
}

//==============================================================================
Supergraph::LaneYawGenerator::LaneYawGenerator(
  const std::shared_ptr<const Supergraph>& graph)
: _graph(graph)
{
  if (const auto* diff_drive = graph->traits().get_differential())
  {
    // We pretend the vehicle is always reversible, because this isn't the place
    // where the reversibility constraint is enforced.
    _constraint = DifferentialDriveConstraint(diff_drive->get_forward(), true);
  }
}

//==============================================================================
std::optional<double> Supergraph::LaneYawGenerator::generate(
  const Entry& key,
  const Storage& /*old_items*/,
  Storage& new_items) const
{
  if (key.orientation == Orientation::Any)
  {
    for (std::size_t j=0; j <= static_cast<std::size_t>(Side::Finish); ++j)
      new_items.insert({{key.lane, Orientation::Any, Side(j)}, std::nullopt});
    return std::nullopt;
  }

  if (!_constraint.has_value())
  {
    for (std::size_t i=0; i <= static_cast<std::size_t>(Orientation::Any); ++i)
    {
      for (std::size_t j=0; j <= static_cast<std::size_t>(Side::Finish); ++j)
        new_items.insert({{key.lane, Orientation(i), Side(j)}, std::nullopt});
    }

    return std::nullopt;
  }

  const auto supergraph = _graph.lock();

  // TODO(MXG): When we have C++20 support, we can label this with [[unlikely]]
  if (!supergraph)
  {
    // This means the supergraph that's being traversed has destructed while
    // this cache is still alive. That's really weird and shouldn't happen.
    // The only reason we keep the graph as a nullptr is
    // 1) to avoid a circular dependency
    // 2) we cannot technically guarantee that the cache's lifecycle will fit
    //    within the supergraph's lifecycle, and throwing an exception is
    //    preferable to Undefined Behavior.
    throw std::runtime_error(
          "[rmf_traffic::agv::planning::Supergraph::EntriesGenerator::generate]"
          " Supergraph died while a EntriesCache was still being used. "
          "Please report this critical bug to the maintainers of rmf_traffic.");
  }

  const auto& original = supergraph->original();
  const auto& lane = original.lanes[key.lane];
  const std::size_t waypoint_index_0 = lane.entry().waypoint_index();
  const std::size_t waypoint_index_1 = lane.exit().waypoint_index();
  const auto& wp0 = original.waypoints[waypoint_index_0];
  const auto& wp1 = original.waypoints[waypoint_index_1];

  const Eigen::Vector2d p0 = wp0.get_location();
  const Eigen::Vector2d p1 = wp1.get_location();
  const double dist = (p1 - p0).norm();
  if (dist <= supergraph->options().translation_thresh)
  {
    for (std::size_t i=0; i <= static_cast<std::size_t>(Orientation::Any); ++i)
    {
      for (std::size_t j=0; j <= static_cast<std::size_t>(Side::Finish); ++j)
        new_items.insert({{key.lane, Orientation(i), Side(j)}, std::nullopt});
    }

    return std::nullopt;
  }

  const Eigen::Vector2d course_vector = (p1 - p0)/dist;
  const auto orientations = _constraint->get_orientations(course_vector);
  for (std::size_t i=0; i < orientations.size(); ++i)
  {
    const auto yaw = orientations[i];
    assert(yaw.has_value());

    for (std::size_t j=0; j <= static_cast<std::size_t>(Side::Finish); ++j)
      new_items.insert({{key.lane, Orientation(i), Side(j)}, yaw});
  }

  return orientations[static_cast<std::size_t>(key.orientation)];
}

//==============================================================================
Supergraph::Supergraph(
  Graph::Implementation original,
  VehicleTraits traits,
  const Interpolate::Options::Implementation& interpolate)
: _original(std::move(original)),
  _traits(std::move(traits)),
  _interpolate(interpolate),
  _floor_changes(find_floor_changes(_original))
{
  if (const auto* diff = _traits.get_differential())
  {
    _constraint = DifferentialDriveConstraint(
          diff->get_forward(), diff->is_reversible());
  }
}

} // namespace planning
} // namespace agv
} // namespace rmf_traffic
