#pragma once
#include "geometry.hpp"
#include <algorithm>
#include <bits/ranges_algo.h>
#include <cstddef>
#include <limits>
#include <optional>
#include <ranges>
#include <span>
#include <variant>

namespace geometry::queries {

template <class... Ts>
struct Multilambda : Ts... {
    using Ts::operator()...;
};

/*
 * Класс для поиска расстояния от точки до фигуры
 *
 * Требуется организовать возможность нахождения расстояния для всех возможных фигур типа-суммы Shape
 */
struct PointToShapeDistanceVisitor {
    Point2D point;

    explicit PointToShapeDistanceVisitor(const Point2D &p) : point(p) {}

    [[nodiscard]] double operator()(const Line &line) const noexcept {

        // Вектор, представляющий направление отрезка (от start к end)
        const Point2D v = line.end - line.start;

        // Вектор от начала отрезка (line.start) до нашей точки
        const Point2D w = point - line.start;

        // Проекция вектора w на вектор v (скалярное произведение)
        // Показывает, насколько точка "проецируется" вдоль отрезка
        const double c1 = w.Dot(v);

        // Если проекция отрицательна, точка находится "перед" началом отрезка
        // Возвращаем расстояние от точки до начала отрезка
        if (c1 <= 0)
            return w.Length();

        // Квадрат длины отрезка (скалярное произведение v на себя)
        const double c2 = v.Dot(v);

        // Если проекция превышает длину отрезка, точка находится "за" концом отрезка
        // Возвращаем расстояние от точки до конца отрезка
        if (c2 <= c1)
            return (point - line.end).Length();

        // Вычисляем параметр b - относительное положение проекции на отрезке
        // 0 = в начале отрезка, 1 = в конце
        const double b = c1 / c2;

        // Находим точку проекции на отрезке
        const Point2D pb = line.start + v * b;

        // Возвращаем расстояние от исходной точки до точки проекции
        return point.DistanceTo(pb);
    }

    [[nodiscard]] double operator()(const Circle &circle) const noexcept {
        // Если point принадлежит окружности, то расстояние 0.0
        if (point.DistanceTo(circle.center_p) <= circle.radius) {
            return 0.0;
        }
        return circle.center_p.DistanceTo(point) - circle.radius;
    }

    [[nodiscard]] double operator()(const Triangle &triangle) const noexcept {
        const auto vertices = triangle.Vertices();
        return getDistance(vertices);
    }

    [[nodiscard]] double operator()(const Polygon &polygon) const noexcept {
        const auto &vertices = polygon.Vertices();
        return getDistance(vertices);
    }

    [[nodiscard]] double operator()(const RegularPolygon &regular_polygon) const noexcept {
        const auto vertices = regular_polygon.Vertices();
        return getDistance(vertices);
    }

    [[nodiscard]] double operator()(const Rectangle &rectangle) const noexcept {

        const auto vertices = rectangle.Vertices();

        if (IsPointInside(vertices)) {
            return 0.0;
        }

        double bottom = rectangle.bottom_left.y;
        double left = rectangle.bottom_left.x;
        double top = rectangle.bottom_left.y + rectangle.height;
        double right = rectangle.bottom_left.x + rectangle.width;

        // Вычисляем расстояние до ближайшей стороны
        const double dx = std::min(std::abs(point.x - left), std::abs(point.x - right));
        const double dy = std::min(std::abs(point.y - bottom), std::abs(point.y - top));

        // Если точка сбоку от прямоугольника
        if (point.x >= left && point.x <= right)
            return dy;
        if (point.y >= bottom && point.y <= top)
            return dx;

        // Если точка в углу - возвращаем евклидово расстояние
        return std::hypot(dx, dy);
    }

private:
    // Вспомогательная функция, проверяет находится ли точка внутри
    // Triangle, Polygon, RegularPolygone используя алгоритм ray casting.
    [[nodiscard]] constexpr bool IsPointInside(const std::span<const Point2D> &vertices) const noexcept {
        // Если полигон имеет меньше 3 вершин, он невалиден
        if (vertices.size() < 3)
            return false;

        // Создаём диапазон пар соседних вершин: (v0,v1), (v1,v2), ..., (vn,v0)
        auto edges = vertices | views::adjacent<2>;  // adjacent<2> даёт пары (current, next)

        // Считаем количество пересечений луча, идущего вправо от точки, с рёбрами полигона
        int crossings = std::ranges::count_if(edges, [&](const auto &edge) {
            const auto &[p1, p2] = edge;  // Текущее ребро (p1 -> p2)

            // Проверяем, находится ли точка между y-координатами ребра
            const bool y_in_range = (p1.y > point.y) != (p2.y > point.y);

            // Если да, вычисляем x-координату пересечения луча с ребром
            if (y_in_range) {
                // Формула пересечения: x = x1 + (x2 - x1) * (y - y1) / (y2 - y1)
                double intersect_x = p1.x + (p2.x - p1.x) * (point.y - p1.y) / (p2.y - p1.y);
                return point.x < intersect_x;  // Пересекает ли луч справа от точки?
            }
            return false;
        });

        // Если число пересечений нечётное, точка внутри полигона (алгоритм ray casting)
        return crossings % 2 == 1;
    }
    // Вспомогательная функция, находит минимальное расстояние между точкой и ближайшим ребром
    // используется для Triangle, Polygon, RegularPolygone
    [[nodiscard]] constexpr double getDistance(const std::span<const Point2D> vertices) const noexcept {

        // Если точка принадлежит фигуре, то расстояние 0.0
        if (IsPointInside(vertices)) {
            return 0.0;
        }

        // Создаём диапазон пар соседних вершин: (v0,v1), (v1,v2), ..., (vn,v0)
        auto edges = vertices | views::adjacent<2>;

        // Находим минимальное расстояние
        return rng::min(edges | views::transform([&](const auto &edge) {
                            return this->operator()(Line{std::get<0>(edge), std::get<1>(edge)});
                        }));
    }
};

/*
 * Класс для поиска расстояния между двумя фигурами
 *
 * Требуется организовать возможность нахождения расстояния только для следующих комбинаций фигур:
 *    - Any    & Point
 *    - Line   & Line
 *    - Circle & Circle
 *
 * Для всех остальных требуется вернуть пустое значение
 */
struct ShapeToShapeDistanceVisitor {

    [[nodiscard]] std::optional<double> operator()(const auto &shape, const Point2D &point) const noexcept {
        return std::visit(PointToShapeDistanceVisitor{point}, shape);
    }

    [[nodiscard]] std::optional<double> operator()(const Point2D &point, const auto &shape) const noexcept {
        return this->operator()(shape, point);
    }

    [[nodiscard]] std::optional<double> operator()(const Line &lhs, const Line &rhs) const noexcept {

        // Если отрезки пересекаются, то расстояние 0.0
        if (LinesIntersect(lhs, rhs)) {
            return 0.0;
        }

        // clang-format off
        // Если отрезки не пересекаются, расстояние - это минимум из расстояний
        // от конечных точек одного отрезка до другого.
        return std::min({PointToShapeDistanceVisitor{lhs.start}(rhs), 
                            PointToShapeDistanceVisitor{lhs.end}(rhs),
                            PointToShapeDistanceVisitor{rhs.start}(lhs), 
                            PointToShapeDistanceVisitor{rhs.end}(lhs)});
        // clang-format on
    }

    [[nodiscard]] std::optional<double> operator()(const Circle &lhs, const Circle &rhs) const noexcept {
        const double d = lhs.Center().DistanceTo(rhs.Center());
        const double r_sum = lhs.radius + rhs.radius;

        if (d > r_sum) {
            return d - r_sum;  // Окружности снаружи друг от друга
        }
        const double r_diff = std::abs(lhs.radius - rhs.radius);
        if (d < r_diff) {
            return 0.0;  // Одна окружность внутри другой
        }
        return 0.0;  // Окружности пересекаются или касаются
    }

    [[nodiscard]] std::optional<double> operator()(const auto &, const auto &) const noexcept { return std::nullopt; }

private:
    [[nodiscard]] constexpr bool LinesIntersect(const Line &lhs, const Line &rhs) const noexcept {

        constexpr auto orientation = [](Point2D p, Point2D q, Point2D r) constexpr noexcept {
            const double val = (q - p).Cross(r - q);
            if (std::abs(val) < std::numeric_limits<double>::epsilon())
                return 0;
            return (val > 0) ? 1 : 2;
        };

        constexpr auto on_segment = [](Point2D p, Point2D q, Point2D r) constexpr noexcept {
            return q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) && q.y <= std::max(p.y, r.y) &&
                   q.y >= std::min(p.y, r.y);
        };

        // Вычисляем ориентации
        const int o1 = orientation(lhs.start, lhs.end, rhs.start);
        const int o2 = orientation(lhs.start, lhs.end, rhs.end);
        const int o3 = orientation(rhs.start, rhs.end, lhs.start);
        const int o4 = orientation(rhs.start, rhs.end, lhs.end);

        // Общий случай пересечения
        if (o1 != o2 && o3 != o4) {
            // Вычисляем точку пересечения
            double t = ((rhs.start.x - rhs.end.x) * (lhs.start.y - rhs.start.y) -
                        (rhs.start.y - rhs.end.y) * (lhs.start.x - rhs.start.x)) /
                       ((rhs.start.y - rhs.end.y) * (lhs.end.x - lhs.start.x) -
                        (rhs.start.x - rhs.end.x) * (lhs.end.y - lhs.start.y));

            Point2D intersection{lhs.start.x + t * (lhs.end.x - lhs.start.x),
                                 lhs.start.y + t * (lhs.end.y - lhs.start.y)};

            // Проверяем, что точка пересечения лежит на обоих отрезках
            if (on_segment(lhs.start, intersection, lhs.end) && on_segment(rhs.start, intersection, rhs.end)) {
                return 0.0;
            }
        }

        // Специальные случаи коллинеарности
        if (o1 == 0 && on_segment(lhs.start, rhs.start, lhs.end))
            return true;
        if (o2 == 0 && on_segment(lhs.start, rhs.end, lhs.end))
            return true;
        if (o3 == 0 && on_segment(rhs.start, lhs.start, rhs.end))
            return true;
        if (o4 == 0 && on_segment(rhs.start, lhs.end, rhs.end))
            return true;

        return false;
    }
};

/*
 * Функции-помощники
 */
inline double DistanceToPoint(const Shape &shape, const Point2D &point) {
    return std::visit(PointToShapeDistanceVisitor{point}, shape);
}

inline BoundingBox GetBoundBox(const Shape &shape) {
    return std::visit([](const auto &s) { return s.BoundBox(); }, shape);
    ;
}

inline double GetHeight(const Shape &shape) {
    return std::visit([](const auto &s) { return s.Center().y + s.Height() / 2; }, shape);
}

inline bool BoundingBoxesOverlap(const Shape &shape1, const Shape &shape2) {
    return GetBoundBox(shape1).Overlaps(GetBoundBox(shape2)).has_value();
}

inline std::optional<double> DistanceBetweenShapes(const Shape &shape1, const Shape &shape2) {
    return std::visit(ShapeToShapeDistanceVisitor{}, shape1, shape2);
}

}  // namespace geometry::queries