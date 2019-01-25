#include <pluginlib/class_list_macros.h>
#include "test_planner/test_planner.h"

namespace test_planner {

	GlobalPlanner::GlobalPlanner() {
	  initialized_ = false;
	}

	GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros) {
	  initialized_ = false;
	  initialize(name, costmap_ros);
	}

	void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros) {

	  if (!initialized_) {
	    costmap_ros_ = costmap_ros;
	    costmap_ = costmap_ros->getCostmap();

	    origin_x_ = costmap_->getOriginX();
	    origin_y_ = costmap_->getOriginY();

	    width_ = costmap_->getSizeInCellsX();
	    height_ = costmap_->getSizeInCellsY();
	    resolution_ = costmap_->getResolution();
	    map_size_ = width_ * height_;


	    for (size_t idy = 0; idy < height_; ++idy) {
	      for (size_t idx = 0; idx < width_; ++idx) {
	        unsigned char cost = costmap_->getCost(idx, idy);
	        cost_.push_back(cost);
	        if (cost == 0) {
	          map_.push_back(true);
	        } else {
	          map_.push_back(false);
	        }
	      }
	    }

	    initialized_ = true;
	  }
	}

	bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped &start,
	                             const geometry_msgs::PoseStamped &goal,
	                             std::vector<geometry_msgs::PoseStamped> &plan) {

	  plan.clear();

	  float start_x = start.pose.position.x;
	  float start_y = start.pose.position.y;
	  FromPositionToIndex(start_x, start_y);

	  size_t current_index = ToIndex(start_x, start_y);

	  float x_end = goal.pose.position.x;
	  float y_end = goal.pose.position.y;
	  FromPositionToIndex(x_end, y_end);

	  std::cout << start_x << ", " << start_y << "\n";
	  size_t goal_index = ToIndex(x_end, y_end);

	  std::vector<bool> visited(map_size_, false);
	  std::priority_queue<std::pair<float, int>,
	                      std::vector<std::pair<float, int> >,
	                      std::greater<std::pair<float, int> > > pq;

	  std::vector<float> dist(map_size_, std::numeric_limits<float>::infinity());
	  std::vector<size_t> prev(map_size_, std::numeric_limits<size_t>::max());

	  pq.push(std::make_pair(0, current_index));
	  dist.at(current_index) = 0;


	  bool found_plan = false;

	  while (!pq.empty()) {
	    size_t u = pq.top().second;
	    int x_u, y_u;
      FromIndex(u, x_u, y_u);
	    pq.pop();

	    std::vector<size_t> adj = {u - 1, u + 1, u - width_, u + width_, u - width_ - 1, u - width_ + 1, u + width_ - 1, u + width_ + 1};

	    for (size_t v : adj) {
	      int x, y;
	      FromIndex(v, x, y);

	      float distance = std::sqrt(std::pow(x - x_u, 2) + std::pow(y - y_u, 2));
	      if (dist.at(u) + distance < dist.at(v) && costmap_->getCost(x, y) == costmap_2d::FREE_SPACE && !visited.at(v)) {
	        dist.at(v) = dist.at(u) + distance;
	        pq.push(std::make_pair(dist.at(v), v));

	        prev.at(v) = u;
	      }
	    }

	    visited.at(u) = true;

	    if (u == goal_index) {
	      std::cout << "Goal index reached\n";
	      found_plan = true;
        break;
      }
	  }

	  if (found_plan) {
	    size_t idx = goal_index;
      while (prev.at(idx) != current_index) {
        if (prev.at(idx) == std::numeric_limits<size_t>::max()) {
          std::cout << "No path found!\n";
          return false;
        }
        idx = prev.at(idx);
        int x, y;
        FromIndex(idx, x, y);
        float x_path = static_cast<float>(x);
        float y_path = static_cast<float>(y);

        FromIndexToPosition(x_path, y_path);
        geometry_msgs::PoseStamped p;
        p.header.frame_id = start.header.frame_id;
        p.pose.position.x = x_path;
        p.pose.position.y = y_path;
        p.pose.orientation.x = 0;
        p.pose.orientation.y = 0;
        p.pose.orientation.z = 0;
        p.pose.orientation.w = 1;

        plan.push_back(p);
      }

      std::reverse(plan.begin(), plan.end());
      plan.insert(plan.begin(), start);
      plan.push_back(goal);
      return true;

	  } else {
	    return false;
	  }

	  return false;
	}

	size_t GlobalPlanner::ToIndex(float x, float y) {
	  return y * width_ + x;
	}

	void GlobalPlanner::FromIndex(size_t index, int &x, int &y) {
	  x = index % width_;
	  y = std::floor(index / width_);
	}

	void GlobalPlanner::FromPositionToIndex(float &x, float &y) {
	  x = static_cast<size_t>((x - origin_x_) / resolution_);
	  y = static_cast<size_t>((y - origin_y_) / resolution_);
	}

	void GlobalPlanner::FromIndexToPosition(float &x, float &y) {
	  x = x * resolution_ + origin_x_;
	  y = y * resolution_ + origin_y_;
	}

	size_t GlobalPlanner::MinDistance(std::vector<float> &distance, std::vector<int> &visited) {

	  float min = std::numeric_limits<float>::max();
	  size_t min_index;
	  size_t remove;

	  for (size_t idx = 0; idx < visited.size(); ++idx) {
	    if (distance.at(visited.at(idx)) < min) {
	      min = distance.at(visited.at(idx));
	      min_index = visited.at(idx);
	      remove = idx;
	    }
	  }
	  visited.erase(visited.begin() + remove - 1);
	  return min_index;
	}

	void GlobalPlanner::GetCoordinate(float &x, float &y) {
	  x = x - origin_x_;
	  y = y - origin_y_;
	}

	size_t GlobalPlanner::GetCellIndex(float x, float y) { // Get the cell index value
	  return (y * width_) + x;
	}

	size_t GlobalPlanner::ConvertToCellIndex(float x, float y) {
	  float new_x = x / resolution_;
	  float new_y = y / resolution_;

	  return GetCellIndex(new_x, new_y);
	}

	size_t GlobalPlanner::GetCost(size_t x, size_t y) {
	  return costmap_->getCost(x, y);
	}
}


PLUGINLIB_EXPORT_CLASS(test_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

