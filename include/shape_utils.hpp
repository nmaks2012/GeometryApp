#pragma once
#include "geometry.hpp"
#include "queries.hpp"
#include <algorithm>
#include <optional>
#include <print>
#include <random>
#include <ranges>
#include <utility>
#include <vector>

namespace geometry::utils {

class ShapeGenerator {
public:
    ShapeGenerator(double min_coord = -100.0, double max_coord = 100.0, double min_size = 1.0, double max_size = 20.0)
        : gen(20), coord_dist(min_coord, max_coord), size_dist(min_size, max_size), sides_dist(3, 12), type_dist(0, 4) {
    }

    Shape GenerateRandomShape() {
        Point2D center{coord_dist(gen), coord_dist(gen)};
        double size = size_dist(gen);

        switch (type_dist(gen)) {
        case 0: {
            Point2D end{center.x + size, center.y + size};
            return Line{center, end};
        }
        case 1: {
            Point2D a{center.x, center.y};
            Point2D b{center.x + size, center.y};
            Point2D c{center.x + size / 2, center.y + size};
            return Triangle{a, b, c};
        }
        case 2: {
            return Rectangle{center, size, size * 0.8};
        }
        case 3: {
            int sides = sides_dist(gen);
            return RegularPolygon{center, size, sides};
        }
        case 4: {
            return Circle{center, size};
        }
        }
        return Circle{center, size};
    }

    std::vector<Shape> GenerateShapes(size_t count) {
        std::vector<Shape> shapes;
        shapes.reserve(count);

        for (auto _ : std::views::iota(0u, count)) {
            shapes.push_back(GenerateRandomShape());
        }

        return shapes;
    }

private:
    std::mt19937 gen;
    std::uniform_real_distribution<double> coord_dist;
    std::uniform_real_distribution<double> size_dist;
    std::uniform_int_distribution<int> sides_dist;
    std::uniform_int_distribution<int> type_dist;
};

std::vector<std::pair<Shape, Shape>> FindAllCollisions(const std::span<Shape> shapes) {

    /*
     * Используйте библиотеку ranges, чтобы найти все коллизии между фигурами методом BoundingBoxesOverlap
     *
     * Также используйте наиболее эффективный метод добавления объектов в collisions
     */
    // clang-format off
     auto indices = views::iota(0u, shapes.size());
     return views::cartesian_product(indices, indices) | 
                                        // Исключение сравнения объекта с самим собой
                                        views::filter([](const auto &p) { return std::get<0>(p) < std::get<1>(p); }) |
                                        // Оставляем только фигуры, в которых есть коллизии
                                        views::filter([&](const auto& pair_idx){
                                            return queries::BoundingBoxesOverlap(shapes[std::get<0>(pair_idx)], shapes[std::get<1>(pair_idx)]); } ) | 
                                        views::transform([&](const auto& pair_idx){
                                            return std::make_pair(shapes[std::get<0>(pair_idx)], shapes[std::get<1>(pair_idx)]);
                                        }) | 
                                        rng::to<std::vector<std::pair<Shape, Shape>>>();
    // clang-format on

}

std::optional<size_t> FindHighestShape(const std::span<Shape> shapes) {

    /*
     * Используйте библиотеку ranges, чтобы найти самую высокую фигуру
     *
     * Важно: использование ручной итерации по фигурам не разрешается
     */
    if (shapes.empty()) {
        return std::nullopt;
    }

    auto indices = views::iota(0u, shapes.size());

    auto max_iterator = rng::max_element(indices, {}, [&](size_t i) { return queries::GetHeight(shapes[i]); });

    return *max_iterator;
}

}  // namespace geometry::utils