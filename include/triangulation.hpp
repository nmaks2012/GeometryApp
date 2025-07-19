#pragma once
#include "geometry.hpp"
#include <algorithm>
#include <format>
#include <set>
#include <vector>

namespace geometry::triangulation {

struct DelaunayTriangle {
    Point2D a, b, c;

    DelaunayTriangle(Point2D a, Point2D b, Point2D c) : a(a), b(b), c(c) {}

    bool ContainsPoint(const Point2D &p) const {
        Point2D center = Circumcenter();
        double radius = Circumradius();
        return center.DistanceTo(p) <= radius + 1e-10;
    }

    Point2D Circumcenter() const {
        double d = 2 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
        if (std::abs(d) < 1e-10) {
            return {(a.x + b.x + c.x) / 3, (a.y + b.y + c.y) / 3};
        }

        double ux = ((a.x * a.x + a.y * a.y) * (b.y - c.y) + (b.x * b.x + b.y * b.y) * (c.y - a.y) +
                     (c.x * c.x + c.y * c.y) * (a.y - b.y)) /
                    d;

        double uy = ((a.x * a.x + a.y * a.y) * (c.x - b.x) + (b.x * b.x + b.y * b.y) * (a.x - c.x) +
                     (c.x * c.x + c.y * c.y) * (b.x - a.x)) /
                    d;

        return {ux, uy};
    }

    double Circumradius() const {
        Point2D center = Circumcenter();
        return center.DistanceTo(a);
    }

    bool SharesEdge(const DelaunayTriangle &other) const {
        std::vector<Point2D> this_points = {a, b, c};
        std::vector<Point2D> other_points = {other.a, other.b, other.c};

        int shared_count = 0;
        for (const Point2D &p1 : this_points) {
            for (const Point2D &p2 : other_points) {
                if (std::abs(p1.x - p2.x) < 1e-10 && std::abs(p1.y - p2.y) < 1e-10) {
                    shared_count++;
                    break;
                }
            }
        }

        return shared_count == 2;
    }

    std::vector<Point2D> vertices() const { return {a, b, c}; }
};

struct Edge {
    Point2D p1, p2;

    Edge(Point2D p1, Point2D p2) : p1(p1), p2(p2) {
        if (p1.x > p2.x || (p1.x == p2.x && p1.y > p2.y)) {
            std::swap(this->p1, this->p2);
        }
    }

    bool operator<(const Edge &other) const {
        if (std::abs(p1.x - other.p1.x) > 1e-10)
            return p1.x < other.p1.x;
        if (std::abs(p1.y - other.p1.y) > 1e-10)
            return p1.y < other.p1.y;
        if (std::abs(p2.x - other.p2.x) > 1e-10)
            return p2.x < other.p2.x;
        return p2.y < other.p2.y;
    }

    bool operator==(const Edge &other) const {
        return std::abs(p1.x - other.p1.x) < 1e-10 && std::abs(p1.y - other.p1.y) < 1e-10 &&
               std::abs(p2.x - other.p2.x) < 1e-10 && std::abs(p2.y - other.p2.y) < 1e-10;
    }
};

inline GeometryResult<std::vector<DelaunayTriangle>> DelaunayTriangulation(std::span<const Point2D> points) {

     if (points.size() < 3) {
        return std::unexpected(GeometryError::InvalidInput);
    }

    // 1. Создаем супер-треугольник, который охватывает все точки
    // Находим границы множества точек
    BoundingBox bbox = {points[0].x, points[0].y, points[0].x, points[0].y};
    for (const auto& p : points) {
        bbox.min_x = std::min(bbox.min_x, p.x);
        bbox.max_x = std::max(bbox.max_x, p.x);
        bbox.min_y = std::min(bbox.min_y, p.y);
        bbox.max_y = std::max(bbox.max_y, p.y);
    }

    // Расширяем границы на 10% для надежности
    const double dx = bbox.max_x - bbox.min_x;
    const double dy = bbox.max_y - bbox.min_y;
    const double delta = std::max(dx, dy) * 1.1;
    const double mid_x = (bbox.min_x + bbox.max_x) / 2.0;
    const double mid_y = (bbox.min_y + bbox.max_y) / 2.0;

    // Вершины супер-треугольника (достаточно большого, чтобы содержать все точки)
    Point2D super1(mid_x - 20 * delta, mid_y - delta);
    Point2D super2(mid_x, mid_y + 20 * delta);
    Point2D super3(mid_x + 20 * delta, mid_y - delta);

    // Инициализируем триангуляцию супер-треугольником
    std::vector<DelaunayTriangle> triangulation;
    triangulation.emplace_back(super1, super2, super3);

    // 2. Последовательно добавляем каждую точку в триангуляцию
    for (const auto& point : points) {
        std::vector<DelaunayTriangle> bad_triangles;
        
        // Находим все "плохие" треугольники, чьи окружности содержат текущую точку
        for (const auto& triangle : triangulation) {
            if (triangle.ContainsPoint(point)) {
                bad_triangles.push_back(triangle);
            }
        }

        // Находим границу полигональной "дырки"
        std::set<Edge> polygon_edges;
        for (const auto& triangle : bad_triangles) {
            // Получаем все ребра текущего треугольника
            std::vector<Edge> edges = {
                Edge(triangle.a, triangle.b),
                Edge(triangle.b, triangle.c),
                Edge(triangle.c, triangle.a)
            };

            // Для каждого ребра проверяем, встречалось ли оно раньше
            for (const auto& edge : edges) {
                auto it = polygon_edges.find(edge);
                if (it == polygon_edges.end()) {
                    polygon_edges.insert(edge); // Добавляем новое ребро
                } else {
                    polygon_edges.erase(it); // Удаляем ребро, если оно уже встречалось
                }
            }
        }

        // Удаляем все плохие треугольники из триангуляции
        triangulation.erase(
            std::remove_if(triangulation.begin(), triangulation.end(),
                [&bad_triangles](const DelaunayTriangle& t) {
                    return std::find_if(bad_triangles.begin(), bad_triangles.end(),
                        [&t](const DelaunayTriangle& bad_t) {
                            return t.a == bad_t.a && t.b == bad_t.b && t.c == bad_t.c;
                        }) != bad_triangles.end();
                }),
            triangulation.end()
        );

        // Создаем новые треугольники из ребер полигона и новой точки
        for (const auto& edge : polygon_edges) {
            triangulation.emplace_back(edge.p1, edge.p2, point);
        }
    }

    // 3. Удаляем все треугольники, связанные с вершинами супер-треугольника
    triangulation.erase(
        std::remove_if(triangulation.begin(), triangulation.end(),
            [super1, super2, super3](const DelaunayTriangle& t) {
                return t.a == super1 || t.a == super2 || t.a == super3 ||
                       t.b == super1 || t.b == super2 || t.b == super3 ||
                       t.c == super1 || t.c == super2 || t.c == super3;
            }),
        triangulation.end()
    );

    return triangulation;
}
}  // namespace geometry::triangulation

template <>
struct std::formatter<geometry::triangulation::DelaunayTriangle> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::triangulation::DelaunayTriangle &t, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "DelaunayTriangle({}, {}, {})", t.a, t.b, t.c);
    }
};
