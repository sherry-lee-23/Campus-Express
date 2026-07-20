#include "types.hpp"
#include "graph.hpp"
#include "queue.hpp"
#include <algorithm>
#include <vector>
#include <functional>
#include <cmath>
#include <unordered_set>

class Scheduler {
private:
    const Graph& graph;
    const Car& car;
    
    std::vector<Package> nearest_neighbor_order(const std::vector<Package>& load, int start_pos,
                                                 std::function<double(const Package&)> priority = nullptr) {
        std::vector<Package> result;
        std::unordered_set<int> delivered;
        int current_pos = start_pos;
        std::vector<Package> remaining = load;
        
        while (!remaining.empty()) {
            double best_score = -INF;
            int best_idx = -1;
            
            for (size_t i = 0; i < remaining.size(); i++) {
                if (delivered.count(remaining[i].id)) continue;
                
                double dist = graph.get_distance(current_pos, remaining[i].dest);
                double score;
                
                if (priority) {
                    score = priority(remaining[i]) / (dist + 1e-9);
                } else {
                    score = 1.0 / (dist + 1e-9);
                }
                
                if (score > best_score) {
                    best_score = score;
                    best_idx = i;
                }
            }
            
            if (best_idx == -1) break;
            
            result.push_back(remaining[best_idx]);
            delivered.insert(remaining[best_idx].id);
            current_pos = remaining[best_idx].dest;
            
            remaining.erase(remaining.begin() + best_idx);
        }
        
        return result;
    }
    
    StrategyResult simulate_t2_with_strategy(std::vector<Package> packages,
                                             std::function<std::vector<Package>(std::vector<Package>&, double)> select_load,
                                             std::function<std::vector<Package>(std::vector<Package>&, int, double)> order_delivery) {
        StrategyResult result;
        result.total_dissatisfaction = 0;
        double current_time = 0;
        
        for (auto& pkg : packages) {
            pkg.delivered = false;
            pkg.delivery_time = 0;
        }
        
        while (true) {
            std::vector<Package*> available;
            for (auto& pkg : packages) {
                if (!pkg.delivered && pkg.arrive_time <= current_time) {
                    available.push_back(&pkg);
                }
            }
            
            if (available.empty()) {
                bool all_delivered = true;
                double min_arrive = INF;
                for (auto& pkg : packages) {
                    if (!pkg.delivered) {
                        all_delivered = false;
                        if (pkg.arrive_time < min_arrive) {
                            min_arrive = pkg.arrive_time;
                        }
                    }
                }
                if (all_delivered) break;
                current_time = min_arrive;
                continue;
            }
            
            std::vector<Package> available_copy;
            for (auto p : available) available_copy.push_back(*p);
            
            std::vector<Package> current_load = select_load(available_copy, current_time);
            
            std::vector<Package> ordered_load = order_delivery(current_load, 0, current_time);
            
            DeliveryRoute route;
            route.path.push_back(0);
            double route_time = 0;
            int current_pos = 0;
            
            for (auto& p : ordered_load) {
                for (auto& pkg : packages) {
                    if (pkg.id == p.id && !pkg.delivered) {
                        double d = graph.get_distance(current_pos, pkg.dest);
                        route_time += d / car.speed;
                        pkg.delivery_time = current_time + route_time;
                        result.total_dissatisfaction += pkg.delivery_time - pkg.arrive_time;
                        current_pos = pkg.dest;
                        route.path.push_back(current_pos);
                        route.package_ids.push_back(pkg.id);
                        pkg.delivered = true;
                        break;
                    }
                }
            }
            
            double return_d = graph.get_distance(current_pos, 0);
            route_time += return_d / car.speed;
            route.path.push_back(0);
            route.time = route_time;
            
            current_time += route_time;
            result.routes.push_back(route);
        }
        
        return result;
    }
    
    StrategyResult simulate_t3_with_strategy(std::vector<Package> packages,
                                             std::function<std::vector<Package>(std::vector<Package>&, double)> select_load,
                                             std::function<std::vector<Package>(std::vector<Package>&, int, double)> order_delivery) {
        StrategyResult result;
        result.total_cost = 0;
        result.overtime_count = 0;
        double current_time = 0;
        
        for (auto& pkg : packages) {
            pkg.delivered = false;
            pkg.delivery_time = 0;
        }
        
        while (true) {
            std::vector<Package*> available;
            for (auto& pkg : packages) {
                if (!pkg.delivered) {
                    available.push_back(&pkg);
                }
            }
            
            if (available.empty()) break;
            
            std::vector<Package> available_copy;
            for (auto p : available) available_copy.push_back(*p);
            
            std::vector<Package> current_load = select_load(available_copy, current_time);
            
            std::vector<Package> ordered_load = order_delivery(current_load, 0, current_time);
            
            DeliveryRoute route;
            route.path.push_back(0);
            double route_time = 0;
            double route_cost = 0;
            int current_pos = 0;
            
            std::vector<double> remaining_weight;
            std::vector<Package*> pkg_ptrs;
            
            for (auto& p : ordered_load) {
                for (auto& pkg : packages) {
                    if (pkg.id == p.id && !pkg.delivered) {
                        pkg_ptrs.push_back(&pkg);
                        remaining_weight.push_back(pkg.weight);
                        break;
                    }
                }
            }
            
            for (size_t i = 0; i < pkg_ptrs.size(); i++) {
                Package* pkg = pkg_ptrs[i];
                double d = graph.get_distance(current_pos, pkg->dest);
                
                double weight_on_board = car.weight;
                for (size_t j = 0; j < remaining_weight.size(); j++) {
                    weight_on_board += remaining_weight[j];
                }
                
                route_cost += d * weight_on_board;
                route_time += d / car.speed;
                
                pkg->delivery_time = current_time + route_time;
                
                if (pkg->delivery_time > pkg->deadline) {
                    result.overtime_count++;
                }
                
                remaining_weight[i] = 0;
                current_pos = pkg->dest;
                route.path.push_back(current_pos);
                route.package_ids.push_back(pkg->id);
                pkg->delivered = true;
            }
            
            double return_d = graph.get_distance(current_pos, 0);
            route_cost += return_d * car.weight;
            route_time += return_d / car.speed;
            route.path.push_back(0);
            route.cost = route_cost;
            route.time = route_time;
            
            result.total_cost += route_cost;
            current_time += route_time;
            result.routes.push_back(route);
        }
        
        return result;
    }
    
public:
    Scheduler(const Graph& g, const Car& c) : graph(g), car(c) {}
    
    StrategyResult t2_baseline_id_order(std::vector<Package> packages) {
        auto select_load = [this](std::vector<Package>& available, double current_time) {
            std::sort(available.begin(), available.end(), [](const Package& a, const Package& b) {
                return a.id < b.id;
            });
            
            std::vector<Package> load;
            double total_weight = 0;
            for (auto& p : available) {
                if (total_weight + p.weight <= car.capacity) {
                    load.push_back(p);
                    total_weight += p.weight;
                }
            }
            return load;
        };
        
        auto order_delivery = [this](std::vector<Package>& load, int start_pos, double current_time) {
            return nearest_neighbor_order(load, start_pos);
        };
        
        return simulate_t2_with_strategy(packages, select_load, order_delivery);
    }
    
    StrategyResult t2_baseline_nearest_first(std::vector<Package> packages) {
        auto select_load = [this](std::vector<Package>& available, double current_time) {
            std::sort(available.begin(), available.end(), [this](const Package& a, const Package& b) {
                return graph.get_distance(0, a.dest) < graph.get_distance(0, b.dest);
            });
            
            std::vector<Package> load;
            double total_weight = 0;
            for (auto& p : available) {
                if (total_weight + p.weight <= car.capacity) {
                    load.push_back(p);
                    total_weight += p.weight;
                }
            }
            return load;
        };
        
        auto order_delivery = [this](std::vector<Package>& load, int start_pos, double current_time) {
            return nearest_neighbor_order(load, start_pos);
        };
        
        return simulate_t2_with_strategy(packages, select_load, order_delivery);
    }
    
    StrategyResult t2_improved_weighted(std::vector<Package> packages) {
        auto select_load = [this](std::vector<Package>& available, double current_time) {
            std::sort(available.begin(), available.end(), [this, current_time](const Package& a, const Package& b) {
                double waiting_time_a = current_time - a.arrive_time;
                double waiting_time_b = current_time - b.arrive_time;
                
                double dist_a = graph.get_distance(0, a.dest);
                double dist_b = graph.get_distance(0, b.dest);
                
                double score_a = waiting_time_a * 0.4 + (100.0 / dist_a) * 0.4 + a.weight * 0.2;
                double score_b = waiting_time_b * 0.4 + (100.0 / dist_b) * 0.4 + b.weight * 0.2;
                
                return score_a > score_b;
            });
            
            std::vector<Package> load;
            double total_weight = 0;
            for (auto& p : available) {
                if (total_weight + p.weight <= car.capacity) {
                    load.push_back(p);
                    total_weight += p.weight;
                }
            }
            return load;
        };
        
        auto order_delivery = [this](std::vector<Package>& load, int start_pos, double current_time) {
            return nearest_neighbor_order(load, start_pos);
        };
        
        return simulate_t2_with_strategy(packages, select_load, order_delivery);
    }
    
    StrategyResult t3_baseline_id_order(std::vector<Package> packages) {
        auto select_load = [this](std::vector<Package>& available, double current_time) {
            std::sort(available.begin(), available.end(), [](const Package& a, const Package& b) {
                return a.id < b.id;
            });
            
            std::vector<Package> load;
            double total_weight = 0;
            for (auto& p : available) {
                if (total_weight + p.weight <= car.capacity) {
                    load.push_back(p);
                    total_weight += p.weight;
                }
            }
            return load;
        };
        
        auto order_delivery = [this](std::vector<Package>& load, int start_pos, double current_time) {
            return nearest_neighbor_order(load, start_pos);
        };
        
        return simulate_t3_with_strategy(packages, select_load, order_delivery);
    }
    
    StrategyResult t3_baseline_nearest_first(std::vector<Package> packages) {
        auto select_load = [this](std::vector<Package>& available, double current_time) {
            std::sort(available.begin(), available.end(), [this](const Package& a, const Package& b) {
                return graph.get_distance(0, a.dest) < graph.get_distance(0, b.dest);
            });
            
            std::vector<Package> load;
            double total_weight = 0;
            for (auto& p : available) {
                if (total_weight + p.weight <= car.capacity) {
                    load.push_back(p);
                    total_weight += p.weight;
                }
            }
            return load;
        };
        
        auto order_delivery = [this](std::vector<Package>& load, int start_pos, double current_time) {
            return nearest_neighbor_order(load, start_pos);
        };
        
        return simulate_t3_with_strategy(packages, select_load, order_delivery);
    }
    
    StrategyResult t3_improved_weighted(std::vector<Package> packages) {
        auto select_load = [this](std::vector<Package>& available, double current_time) {
            std::sort(available.begin(), available.end(), [this, current_time](const Package& a, const Package& b) {
                double dist_a = graph.get_distance(0, a.dest);
                double dist_b = graph.get_distance(0, b.dest);
                
                double time_left_a = a.deadline - (current_time + dist_a / car.speed);
                double time_left_b = b.deadline - (current_time + dist_b / car.speed);
                
                double urgency_a = (time_left_a > 0) ? (1.0 / time_left_a) : 1e9;
                double urgency_b = (time_left_b > 0) ? (1.0 / time_left_b) : 1e9;
                
                double score_a = urgency_a * 40 + (100.0 / dist_a) * 30 + a.weight * 30;
                double score_b = urgency_b * 40 + (100.0 / dist_b) * 30 + b.weight * 30;
                
                return score_a > score_b;
            });
            
            std::vector<Package> load;
            double total_weight = 0;
            for (auto& p : available) {
                if (total_weight + p.weight <= car.capacity) {
                    load.push_back(p);
                    total_weight += p.weight;
                }
            }
            return load;
        };
        
        auto order_delivery = [this](std::vector<Package>& load, int start_pos, double current_time) {
            return nearest_neighbor_order(load, start_pos, [](const Package& p) {
                return p.weight;
            });
        };
        
        return simulate_t3_with_strategy(packages, select_load, order_delivery);
    }
    
    StrategyResult t3_cost_optimized(std::vector<Package> packages) {
        auto select_load = [this](std::vector<Package>& available, double current_time) {
            std::sort(available.begin(), available.end(), [this](const Package& a, const Package& b) {
                double dist_a = graph.get_distance(0, a.dest);
                double dist_b = graph.get_distance(0, b.dest);
                double score_a = a.weight / dist_a;
                double score_b = b.weight / dist_b;
                return score_a > score_b;
            });
            
            std::vector<Package> load;
            double total_weight = 0;
            for (auto& p : available) {
                if (total_weight + p.weight <= car.capacity) {
                    load.push_back(p);
                    total_weight += p.weight;
                }
            }
            return load;
        };
        
        auto order_delivery = [this](std::vector<Package>& load, int start_pos, double current_time) {
            return nearest_neighbor_order(load, start_pos, [](const Package& p) {
                return p.weight;
            });
        };
        
        return simulate_t3_with_strategy(packages, select_load, order_delivery);
    }
    
    StrategyResult run_strategy(std::vector<Package> packages, StrategyType type, bool is_t3) {
        switch (type) {
            case StrategyType::BASELINE_ID_ORDER:
                return is_t3 ? t3_baseline_id_order(packages) : t2_baseline_id_order(packages);
            case StrategyType::BASELINE_NEAREST_FIRST:
                return is_t3 ? t3_baseline_nearest_first(packages) : t2_baseline_nearest_first(packages);
            case StrategyType::IMPROVED_WEIGHTED_SCORING:
                return is_t3 ? t3_improved_weighted(packages) : t2_improved_weighted(packages);
            default:
                return is_t3 ? t3_baseline_id_order(packages) : t2_baseline_id_order(packages);
        }
    }
};
