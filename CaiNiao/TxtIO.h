#ifndef TXT_IO_H
#define TXT_IO_H

#include <string>
#include <vector>
#include "LGraph.h" 
#include "Delivery.h" 

namespace TxtIO {

bool readMap(const std::string& filepath,
             std::vector<Graph::LocationInfo>& locations,
             std::vector<Graph::EdgeNode>& edges);

bool readPackages(const std::string& filepath,
                  std::vector<Delivery::Package>& packages);

bool readCar(const std::string& filepath,
             Delivery::Car& car);

} // namespace TxtIO

#endif