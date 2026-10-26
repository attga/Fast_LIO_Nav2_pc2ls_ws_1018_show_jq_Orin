#include "ml_scan_context/database.hpp"
#include <fstream>
#include <iostream>
#include <random>
#include <stdexcept>

using namespace ml_scan_context;
void check(bool value) { if (!value) throw std::runtime_error("test check failed"); }
template<class F> void rejects(F action) { bool threw = false; try { action(); } catch (const std::exception &) { threw = true; } check(threw); }
int main() {
  try {
    PolarContext context;
    check(context.describe({}).isZero());
    check(!std::isfinite(context.compare(context.describe({}), context.describe({})).distance));
    Points sparse(4, Eigen::Vector3d(4, 0, -1)); check(context.describe(sparse).isZero());
    sparse.push_back(sparse.front()); check(context.describe(sparse)(1, 0) == -1);
    sparse.push_back(Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()));
    check(context.describe(sparse)(1, 0) == -1);
    rejects([] { PolarContext invalid({0, 60, 80, 5}); });
    rejects([&] { context.compare(Descriptor::Zero(1, 1), context.describe({})); });
    // 缓存持有自己的快照，外部描述子修改不能污染它。
    Descriptor original = Descriptor::Ones(20, 60);
    const auto prepared = context.prepare(original);
    original.setZero();
    check(context.comparePrepared(prepared, prepared).distance < 1e-12);
    check(!std::isfinite(context.comparePrepared(context.prepare(original), prepared).distance));
    rejects([&] { context.comparePrepared(PreparedDescriptor{}, prepared); });
    PolarContext other_shape({10, 60, 80, 5});
    rejects([&] { context.comparePrepared(other_shape.prepare(Descriptor::Ones(10, 60)), prepared); });
    std::mt19937 random(42); std::uniform_real_distribution<double> xy(-30, 30), z(0, 10);
    Points map; for (int i = 0; i < 50000; ++i) map.emplace_back(xy(random), xy(random), z(random));
    const auto rotation = Eigen::AngleAxisd(std::acos(-1.0)/3, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Points query; for (const auto & p : map) query.push_back(rotation * (p - Eigen::Vector3d(-2, -2, 0)));
    DatabaseConfig config; config.preprocessing_id = "test-prepared-xyz";
    Database db(config); db.add(816, map); db.add(1512, map);
    rejects([&] { db.add(816, map); }); check(db.size() == 2);
    auto result = db.query(query); check(result.size() == 2 && result[0].keyframe_id == 816);
    check(result[0].descriptor_distance < 1e-12);
    for (std::size_t i = 0; i < map.size(); i += 100)
      check((result[0].T_keyframe_query * query[i] - map[i]).norm() < 1e-10);
    for (int threads : {1, 2, 4}) {
      QueryOptions parallel; parallel.threads = threads;
      const auto actual = db.query(query, parallel);
      check(actual.size() == result.size());
      for (std::size_t i = 0; i < actual.size(); ++i) {
        check(actual[i].keyframe_id == result[i].keyframe_id);
        check(actual[i].descriptor_distance == result[i].descriptor_distance);
        check(actual[i].T_keyframe_query.matrix() == result[i].T_keyframe_query.matrix());
      }
      parallel.allowed_keyframes = {1512};
      check(db.query(query, parallel).at(0).keyframe_id == 1512);
    }
    QueryOptions invalid_threads; invalid_threads.threads = 0;
    rejects([&] { db.query(query, invalid_threads); });
    QueryOptions options; options.allowed_keyframes = {1512};
    check(db.query(query, options).at(0).keyframe_id == 1512);
    options.allowed_keyframes = {999}; check(db.query(query, options).empty());
    check(db.query({}).empty()); options.top_k = 0; check(db.query(query, options).empty());
    const auto path = std::filesystem::temp_directory_path() / ("mlpcdb-test-" + std::to_string(std::random_device{}()));
    db.save(path); Database restored(config); restored.load(path); check(restored.size() == 2);
    auto after = restored.query(query); check((after[0].T_keyframe_query.matrix() - result[0].T_keyframe_query.matrix()).norm() == 0);
    auto different = config; different.preprocessing_id = "other"; Database wrong(different);
    rejects([&] { wrong.load(path); });
    { std::ofstream file(path); file << "MLPCDB 1\n20 60 80 5\n"; }
    rejects([&] { restored.load(path); }); check(restored.size() == 2);
    const auto preserved = restored.query(query);
    check(preserved[0].descriptor_distance == result[0].descriptor_distance);
    check(preserved[0].keyframe_id == result[0].keyframe_id);
    std::filesystem::remove(path);
    std::cout << "PASS: descriptors, transforms, unique keyframes, filtering, IO roundtrip and rejection\n";
  } catch (const std::exception & error) { std::cerr << error.what() << '\n'; return 1; }
}
