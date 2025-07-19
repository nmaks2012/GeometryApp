#include "convex_hull.hpp"
#include "geometry.hpp"
#include <algorithm>
#include <expected>

namespace geometry::convex_hull {

double CrossProduct(Point2D p1, Point2D middle, Point2D p2) {
    auto new_p1 = p1 - middle;
    auto new_p2 = p2 - middle;
    return new_p1.Cross(new_p2);
}

std::expected<Polygon, GeometryError> GrahamScan(const std::span<Shape> shapes) {

    std::vector<Point2D> points;
    rng::for_each(shapes, [&points](const auto &shape) {
        std::visit(
            [&points](const auto &s) {
                const auto &vertices = s.Vertices();
                rng::copy(vertices, std::back_inserter(points));
            },
            shape);
    });

    // Удаление дубликатов
    rng::sort(points, {}, [](const Point2D &p) { return std::tie(p.x, p.y); });
    const auto [first, last] = rng::unique(points);
    points.erase(first, last);

    if (points.size() < 3) {
        return std::unexpected(GeometryError::DegenrateCase);
    }

    // Находим опорную точку
    const auto pivot_it = rng::min_element(points, {}, [](const Point2D &p) { return std::tie(p.y, p.x); });
    std::iter_swap(points.begin(), pivot_it);
    const Point2D &pivot = points.front();

    // Сортировка по полярному углу
    rng::sort(views::drop(points, 1), [&pivot](const Point2D &a, const Point2D &b) {
        const auto new_a = a - pivot;
        const auto new_b = b - pivot;
        const double cross = new_a.Cross(new_b);
        return std::abs(cross) < 1e-9 ? pivot.DistanceTo(a) < pivot.DistanceTo(b) : cross > 0;
    });

    // Построение выпуклой оболочки
    StackForGrahamScan hull;
    hull.Push(points[0]);
    hull.Push(points[1]);

    rng::for_each(views::drop(points, 2), [&hull](const Point2D &point) {
        while (hull.Size() > 1) {
            const Point2D &top = hull.Top();
            const Point2D &next_top = hull.NextToTop();
            const auto cross = (top - next_top).Cross(point - next_top);
            if (cross <= 0) {
                hull.Pop();
            } else {
                break;
            }
        }
        hull.Push(point);
    });

    if (hull.Size() < 3) {
        return std::unexpected(GeometryError::DegenrateCase);
    }

    return Polygon(std::move(hull).Extract());
}

}  // namespace geometry::convex_hull