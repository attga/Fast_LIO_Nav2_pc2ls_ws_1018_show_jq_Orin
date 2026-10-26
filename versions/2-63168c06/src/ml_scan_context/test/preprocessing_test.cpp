#include "ml_scan_context/preprocessing.hpp"
#include <iostream>
#include <stdexcept>
using namespace ml_scan_context;
void check(bool ok) { if (!ok) throw std::runtime_error("preprocessing check failed"); }
template<class F> void rejects(F f) { bool threw=false;try { f(); } catch (const std::exception &) { threw=true; }check(threw); }
int main() { try {
  Preprocessor process; check(process.apply({}).empty());
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  auto result=process.apply({{-0.1,0,0},{0.1,0,0},{0.3,0,0},{60,0,0},{0,-60,0},{nan,0,0},{0,0,nan}});
  check(result.size()==2);check((result[0]-Eigen::Vector3d(-0.1,0,2)).norm()<1e-14);
  check((result[1]-Eigen::Vector3d(0.2,0,2)).norm()<1e-14);
  auto boundary=process.apply({{0.499,0,0},{0.5,0,0},{-0.5,0,0},{-0.501,0,0}});check(boundary.size()==4);
  Preprocessor shifted({60,0.2,0.5});check(shifted.apply({{0,0,0.2},{0,0,0.4}}).size()==2);
  rejects([] { Preprocessor bad({60,2,0}); });rejects([&] { process.identifier(""); });
  rejects([&] { process.apply({{0,0,1e300}}); });
  check(process.identifier("imu")!=Preprocessor({60,1,0.5}).identifier("imu"));
  check(process.identifier("imu")!=process.identifier("base_link"));
  std::cout<<"PASS preprocessing: finite/range/boundary/centroid/order/offset/config\n";
} catch(const std::exception &e) { std::cerr<<e.what()<<'\n';return 1; } }
