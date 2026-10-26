#include "ml_scan_context/database.hpp"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <locale>
#include <stdexcept>
#include <unordered_set>

namespace ml_scan_context {
void Database::save(const std::filesystem::path & path) const {
  auto temporary = path;
  temporary += ".tmp." + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  try {
    std::ofstream out(temporary); out.imbue(std::locale::classic());
    out.exceptions(std::ios::failbit | std::ios::badbit);
    const auto & p = config_.polar;
    out << std::setprecision(17) << "MLPCDB 1\n" << p.rings << ' ' << p.sectors << ' '
        << p.radius << ' ' << p.minimum_points << '\n' << std::quoted(config_.preprocessing_id) << '\n'
        << config_.viewpoints.size() << '\n';
    for (const auto & v : config_.viewpoints) out << v.x() << ' ' << v.y() << '\n';
    out << frames_.size() << '\n';
    for (const auto & frame : frames_) {
      out << frame.id << '\n';
      for (const auto & descriptor : frame.descriptors) {
        for (Eigen::Index i = 0; i < descriptor.size(); ++i) out << descriptor.data()[i] << ' ';
        out << '\n';
      }
    }
    out.close();
    std::filesystem::rename(temporary, path);
  } catch (...) {
    std::error_code ignored; std::filesystem::remove(temporary, ignored); throw;
  }
}
void Database::load(const std::filesystem::path & path) {
  std::ifstream in(path); in.imbue(std::locale::classic());
  auto require = [&](bool condition) {
    if (!condition || !in) throw std::runtime_error("invalid or incompatible MLPCDB: " + path.string());
  };
  std::string magic, preprocessing; int version = 0; PolarConfig p;
  in >> magic >> version >> p.rings >> p.sectors >> p.radius >> p.minimum_points >> std::quoted(preprocessing);
  const auto & expected = config_.polar;
  require(magic == "MLPCDB" && version == 1 && p.rings == expected.rings &&
    p.sectors == expected.sectors && p.radius == expected.radius &&
    p.minimum_points == expected.minimum_points && preprocessing == config_.preprocessing_id);
  std::size_t views = 0; in >> views; require(views == config_.viewpoints.size());
  for (const auto & v : config_.viewpoints) {
    double x = 0, y = 0; in >> x >> y; require(x == v.x() && y == v.y());
  }
  std::size_t count = 0; in >> count; require(count <= 1000000);
  std::vector<Frame> loaded;
  std::unordered_set<KeyframeId> ids;
  for (std::size_t i = 0; i < count; ++i) {
    Frame frame{}; in >> frame.id; require(ids.insert(frame.id).second);
    for (std::size_t j = 0; j < views; ++j) {
      Descriptor descriptor = Descriptor::Zero(p.rings, p.sectors);
      for (Eigen::Index k = 0; k < descriptor.size(); ++k) in >> descriptor.data()[k];
      require(descriptor.allFinite());
      frame.prepared.push_back(context_.prepare(descriptor));
      frame.descriptors.push_back(std::move(descriptor));
    }
    loaded.push_back(std::move(frame));
  }
  in >> std::ws;
  if (!in.eof()) throw std::runtime_error("trailing data in MLPCDB");
  frames_.swap(loaded);
}
}  // namespace ml_scan_context
