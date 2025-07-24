#pragma once
#include "geometry.hpp"
#include <cmath>
#include <expected>
#include <optional>
#include <variant>

namespace geometry::intersections {

/*
 * Класс для поиска пересечений между двумя фигурами
 *
 * Требуется организовать возможность нахождения пересечений только для следующих комбинаций фигур:
 *    - Line   & Line
 *    - Line   & Circle
 *    - Circle & Line
 *    - Circle & Circle
 *
 * Для всех остальных требуется выбросить исключение std::logic_error
 */
class IntersectionVisitor {
public:
    using ResultType = std::expected<std::optional<Point2D::PointsContainer>, GeometryError>;

    ResultType operator()(const Line &lhs, const Line &rhs) noexcept { return findLinesIntersection(lhs, rhs); }

    ResultType operator()(const Line &lhs, const Circle &rhs) noexcept { return this->operator()(rhs, lhs); }

    ResultType operator()(const Circle &lhs, const Line &rhs) noexcept { return findCircleLineIntersection(lhs, rhs); }

    ResultType operator()(const Circle &lhs, const Circle &rhs) noexcept { return findCircleIntersection(lhs, rhs); }

    ResultType operator()(const auto &, const auto &) noexcept { return std::unexpected(GeometryError::Unsupported); }

private:
    // Проверяет, лежит ли точка p на отрезке
    [[nodiscard]] bool isPointOnLine(const Point2D &p, const Line &line) {
        return std::min(line.start.x, line.end.x) <= p.x && p.x <= std::max(line.start.x, line.end.x) &&
               std::min(line.start.y, line.end.y) <= p.y && p.y <= std::max(line.start.y, line.end.y);
    }
    // Находит точку пересечения двух Line (если она есть)
    [[nodiscard]] std::optional<Point2D::PointsContainer> findLinesIntersection(const Line &lhs, const Line &rhs) {
        const auto &[p1, p2] = lhs;
        const auto &[p3, p4] = rhs;

        // Вычисляем векторы и коэффициенты
        const double a1 = p2.y - p1.y, b1 = p1.x - p2.x, c1 = p2.x * p1.y - p1.x * p2.y;
        const double a2 = p4.y - p3.y, b2 = p3.x - p4.x, c2 = p4.x * p3.y - p3.x * p4.y;

        const double det = a1 * b2 - a2 * b1;
        if (std::abs(det) < 1e-10) {
            return std::nullopt;  // Параллельны или совпадают
        }

        // Точка пересечения Line
        const Point2D p{(b1 * c2 - b2 * c1) / det, (a2 * c1 - a1 * c2) / det};

        // Проверяем что точка принадлежит обоим линиям
        if (isPointOnLine(p, lhs) && isPointOnLine(p, rhs)) {
            return Point2D::PointsContainer{p};
        }

        return std::nullopt;
    }

    // Возвращает точки пересечения двух Circle (0, 1 или 2 точки)
    [[nodiscard]] std::optional<Point2D::PointsContainer> findCircleIntersection(const Circle &c1, const Circle &c2) {

        Point2D::PointsContainer intersections;

        const double dx = c2.Center().x - c1.Center().x;
        const double dy = c2.Center().y - c1.Center().y;
        const double d = std::hypot(dx, dy);  // Расстояние между центрами

        // Окружности не пересекаются
        if (d > c1.radius + c2.radius || d < std::abs(c1.radius - c2.radius)) {
            return std::nullopt;  // Пустой вектор
        }

        // Окружности совпадают (бесконечно много точек)
        if (d == 0.0 && c1.radius == c2.radius) {
            return std::nullopt;  // Можно считать, что точек нет (или все точки)
        }

        // Вычисляем коэффициенты
        const double a = (c1.radius * c1.radius - c2.radius * c2.radius + d * d) / (2 * d);
        const double h = std::sqrt(c1.radius * c1.radius - a * a);

        // Точка P2 (база для вычисления пересечений)
        const Point2D P2{c1.Center().x + (a / d) * dx, c1.Center().y + (a / d) * dy};

        // Добавляем точки пересечения (если h > 0, их две)
        if (h > 1e-10) {  // Проверка на ненулевое h (избегаем деления на 0)
            intersections.push_back({P2.x + (h / d) * dy, P2.y - (h / d) * dx});
            intersections.push_back({P2.x - (h / d) * dy, P2.y + (h / d) * dx});
        } else {  // Касание (1 точка)
            intersections.push_back(P2);
        }

        if (intersections.size()) {
            return intersections;
        }
        return std::nullopt;
    }

    // Находит пересечение Circle и Line
    [[nodiscard]] std::optional<Point2D::PointsContainer> findCircleLineIntersection(const Circle &circle, const Line &line) {
        std::vector<Point2D> intersections;
        const auto &[x1, y1] = line.start;
        const auto &[x2, y2] = line.end;
        const auto &[a, b] = circle.Center();
        const double r = circle.radius;

        // Уравнение прямой: (y2 - y1)x - (x2 - x1)y + (x2 y1 - x1 y2) = 0
        const double A = y2 - y1;
        const double B = x1 - x2;
        const double C = x2 * y1 - x1 * y2;

        // Если прямая вертикальная (B = 0)
        if (std::abs(B) < 1e-10) {
            const double x = -C / A;
            const double D = r * r - (x - a) * (x - a);
            if (D >= 0) {
                const double y1 = b + std::sqrt(D);
                const double y2 = b - std::sqrt(D);
                Point2D p1{x, y1}, p2{x, y2};
                if (isPointOnLine(p1, line))
                    intersections.push_back(p1);
                if (D > 0 && isPointOnLine(p2, line))
                    intersections.push_back(p2);
            }
            return intersections;
        }

        // Общий случай (y = kx + m)
        const double k = -A / B;
        const double m = -C / B;

        // Квадратное уравнение: (1 + k²)x² + 2(k(m - b) - a)x + (a² + (m - b)² - r²) = 0
        const double coeffA = 1 + k * k;
        const double coeffB = 2 * (k * (m - b) - a);
        const double coeffC = a * a + (m - b) * (m - b) - r * r;

        const double D = coeffB * coeffB - 4 * coeffA * coeffC;

        if (D >= 0) {
            const double sqrtD = std::sqrt(D);
            const double x1 = (-coeffB + sqrtD) / (2 * coeffA);
            const double x2 = (-coeffB - sqrtD) / (2 * coeffA);
            const double y1 = k * x1 + m;
            const double y2 = k * x2 + m;
            Point2D p1{x1, y1}, p2{x2, y2};
            if (isPointOnLine(p1, line))
                intersections.push_back(p1);
            if (D > 0 && isPointOnLine(p2, line))
                intersections.push_back(p2);
        }

        if (intersections.size()) {
            return intersections;
        }
        return std::nullopt;
    }
};

[[nodiscard]] inline std::optional<Point2D::PointsContainer> GetIntersectPoint(const Shape &shape1, const Shape &shape2) noexcept {
    auto result = std::visit(IntersectionVisitor{}, shape1, shape2);
    if(result.has_value()){
        // result.value() == доступ к optional внутри expected
        // result->value() == доступ к Point2D::PointsContainer внутни optional внутри expected
        return result.value();
    }
    return std::nullopt;
}

}  // namespace geometry::intersections