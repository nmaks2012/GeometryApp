#pragma once
#include <algorithm>
#include <array>
#include <bits/ranges_algo.h>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <expected>
#include <format>
#include <functional>
#include <initializer_list>
#include <iterator>
#include <numbers>
#include <optional>
#include <print>
#include <ranges>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace rng = std::ranges;
namespace views = std::ranges::views;

namespace geometry {

/*
 * Добавьте к методам класса Point2D и Lines2DDyn все необходимые аттрибуты и спецификаторы
 * Важно: Возвращаемый тип и принимаемые аргументы менять не нужно
 */
struct Point2D {
    using PointsContainer = std::vector<Point2D>;
    double x, y;

    constexpr Point2D() : x(0), y(0) {}
    constexpr Point2D(double x, double y) : x(x), y(y) {}

    // Comparison
    [[nodiscard]] constexpr bool operator<(const Point2D &other) const noexcept { return x < other.x && y < other.y; }
    [[nodiscard]] constexpr bool operator==(const Point2D &other) const noexcept {
        return x == other.x && y == other.y;
    }

    // Binary math operators
    [[nodiscard]] constexpr Point2D operator+(const Point2D &other) const noexcept {
        return {x + other.x, y + other.y};
    }
    [[nodiscard]] constexpr Point2D operator-(const Point2D &other) const noexcept {
        return {x - other.x, y - other.y};
    }
    [[nodiscard]] constexpr Point2D operator*(double value) const noexcept { return {x * value, y * value}; }
    [[nodiscard]] constexpr Point2D operator/(double value) const noexcept { return {x / value, y / value}; }

    // Binary geometry operations
    [[nodiscard]] constexpr double Dot(const Point2D &other) const noexcept { return x * other.x + y * other.y; }
    [[nodiscard]] constexpr double Cross(const Point2D &other) const noexcept { return x * other.y - y * other.x; }
    [[nodiscard]] constexpr double Length() const noexcept { return std::sqrt(x * x + y * y); }
    [[nodiscard]] constexpr double DistanceTo(const Point2D &other) const noexcept { return (*this - other).Length(); }

    [[nodiscard]] constexpr Point2D Normalize() const noexcept {
        const double len = Length();
        return len > 0 ? Point2D{x / len, y / len} : Point2D{0, 0};
    }
};



    template <size_t N>
    struct Lines2D {
        std::array<double, N> x;
        std::array<double, N> y;
    };

    struct Lines2DDyn {
        constexpr Lines2DDyn(Point2D::PointsContainer points) {
            Reserve(points.size() + 1);
            rng::for_each(points, [&](const auto &point) { PushBack(point); });
        }
        Lines2DDyn() = default;
        std::vector<double> x;
        std::vector<double> y;

        constexpr void Reserve(size_t n) {
            x.reserve(n);
            y.reserve(n);
        }
        constexpr void PushBack(Point2D p) {
            x.push_back(p.x);
            y.push_back(p.y);
        }
        constexpr void PushBack(double px, double py) {
            x.push_back(px);
            y.push_back(py);
        }
        [[nodiscard]] constexpr Point2D Front() { return {x.front(), y.front()}; }
    };

    struct BoundingBox {
        double min_x, min_y, max_x, max_y;

        [[nodiscard]] constexpr double Height() const noexcept { return max_y - min_y; }

        [[nodiscard]] constexpr double Width() const noexcept { return max_x - min_x; }

        [[nodiscard]] constexpr Point2D Center() const noexcept { return {(max_x - min_x) / 2, (max_y - min_y) / 2}; }

        // +-----------+
        // |           |
        // | min_x+----+------+
        // |      |    |max_y |
        // |      |    |      |
        // +------+----+      +
        //   min_y|     max_x |
        //        |           |
        //        +-----------+
        [[nodiscard]] constexpr std::optional<BoundingBox> Overlaps(const BoundingBox &other) const noexcept {

            bool x_overlap = (max_x >= other.min_x) && (other.max_x >= min_x);
            bool y_overlap = (max_y >= other.min_y) && (other.max_y >= min_y);

            if (x_overlap && y_overlap) {
                return BoundingBox{std::max(min_x, other.min_x), std::max(min_y, other.min_y),
                                   std::min(max_x, other.max_x), std::min(max_y, other.max_y)};
            }

            return std::nullopt;
        }
        [[nodiscard]] constexpr auto operator<=>(const BoundingBox&) const = default;
    };

    struct Line {
        Point2D start, end;

        [[nodiscard]] constexpr double Length() const noexcept { return start.DistanceTo(end); }
        [[nodiscard]] constexpr Point2D Direction() const noexcept { return (start - end).Normalize(); }

        [[nodiscard]] constexpr BoundingBox BoundBox() const noexcept {
            return {std::min(start.x, end.x), std::min(start.y, end.y), std::max(start.x, end.x),
                    std::max(start.y, end.y)};
        }
        [[nodiscard]] constexpr double Height() const noexcept { return end.y - start.y; }
        [[nodiscard]] constexpr Point2D Center() const noexcept {
            return {(start.x + end.x) / 2, (start.y + end.y) / 2};
        }
        [[nodiscard]] constexpr Point2D::PointsContainer Vertices() const noexcept { return {start, end}; }
        [[nodiscard]] constexpr Lines2D<2> Lines() const noexcept { return {{start.x, end.x}, {start.y, end.y}}; }
    };

    struct Triangle {
        Point2D a, b, c;

        //
        // Обратите внимание! В методе Lines(), в отличие от Vertices(), координаты точек замыкаются на начало:
        // a.x, b.x, c.x а затем идёт снова первая вершина a.x
        //
        // Это необходимо для правильного рисования фигур через gnuplot, который формирует линии используя пары точек.
        // В случае с  Triangle будут составлены такие пары точек:
        //      - { a, b }
        //      - { b, c }
        //      - { c, a }
        //
        [[nodiscard]] constexpr Point2D::PointsContainer Vertices() const noexcept { return {a, b, c}; }
        [[nodiscard]] constexpr Lines2D<4> Lines() const noexcept {
            return {{a.x, b.x, c.x, a.x}, {a.y, b.y, c.y, a.y}};
        }

        [[nodiscard]] constexpr double Area() const noexcept {
            return 0.5 * std::abs((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
        }
        [[nodiscard]] constexpr double Height() const noexcept {
            double base_length = sqrt(std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2));
            return (Area() * 2) / base_length;
        }
        [[nodiscard]] constexpr Point2D Center() const noexcept {
            return {(a.x + b.x + c.x) / 3, (a.y + b.y + c.y) / 3};
        }

        [[nodiscard]] constexpr BoundingBox BoundBox() const noexcept {
            return {std::min({a.x, b.x, c.x}), std::min({a.y, b.y, c.y}), std::max({a.x, b.x, c.x}),
                    std::max({a.y, b.y, c.y})};
        }
    };

    struct Rectangle {
        Point2D bottom_left;
        double width, height;

        [[nodiscard]] constexpr double Area() const noexcept { return width * height; }
        [[nodiscard]] constexpr double Height() const noexcept { return height; }
        [[nodiscard]] constexpr Point2D Center() const noexcept {
            return {bottom_left.x + width / 2, bottom_left.y + height / 2};
        }
        [[nodiscard]] constexpr BoundingBox BoundBox() const noexcept {
            return {bottom_left.x, bottom_left.y, bottom_left.x + width, bottom_left.y + height};
        }
        [[nodiscard]] constexpr Point2D::PointsContainer Vertices() const noexcept {
            return {bottom_left,
                    {bottom_left.x + width, bottom_left.y},
                    {bottom_left.x, bottom_left.y + height},
                    {bottom_left.x + width, bottom_left.y + height}};
        }
        [[nodiscard]] const Lines2D<5> Lines() const noexcept {
            return {{bottom_left.x, bottom_left.x, bottom_left.x + width, bottom_left.x + width, bottom_left.x},
                    {bottom_left.y, bottom_left.y + height, bottom_left.y + height, bottom_left.y, bottom_left.y}};
        }
    };

    struct RegularPolygon {
        Point2D center_p;
        double radius;
        size_t sides;

        constexpr RegularPolygon(Point2D center, double radius, int sides)
            : center_p(center), radius(radius), sides(sides) {}

        [[nodiscard]] constexpr Point2D::PointsContainer Vertices() const noexcept {

            return views::iota(0u, sides) | views::transform([&](auto idx) {
                       const double angle = 2 * std::numbers::pi * idx / sides;
                       return Point2D{center_p.x + radius * std::cos(angle), center_p.y + radius * std::sin(angle)};
                   }) |
                   rng::to<Point2D::PointsContainer>();
        }
        [[nodiscard]] constexpr double Height() const noexcept {

            if (sides % 2 == 0) {
                // Для чётного числа сторон — 2 * апофему
                return 2 * radius * std::cos(std::numbers::pi / sides);
            } else {
                // Для нечётного — радиус + апофема (максимальное расстояние)
                return radius * (1 + std::cos(std::numbers::pi / sides));
            }
        }
        [[nodiscard]] constexpr Point2D Center() const noexcept { return center_p; }

        [[nodiscard]] constexpr BoundingBox BoundBox() const noexcept {

            Point2D::PointsContainer verticies = Vertices();

            auto [x_min, x_max] = rng::minmax_element(verticies, {}, &Point2D::x);
            auto [y_min, y_max] = rng::minmax_element(verticies, {}, &Point2D::y);

            return {x_min->x, y_min->y, x_max->x, y_max->y};
        }
        [[nodiscard]] constexpr Lines2DDyn Lines() const noexcept {
            Lines2DDyn lines = {Vertices()};
            lines.PushBack(lines.Front());
            return lines;
        }
    };

    struct Circle {
        Point2D center_p;
        double radius;

        constexpr Circle(Point2D center, double radius) : center_p(center), radius(radius) {}

        [[nodiscard]] constexpr BoundingBox BoundBox() const noexcept {
            return {center_p.x - radius, center_p.y - radius, center_p.x + radius, center_p.y + radius};
        }
        [[nodiscard]] constexpr double Height() const noexcept { return 2 * radius; }
        [[nodiscard]] constexpr Point2D Center() const noexcept { return center_p; }

        //
        // Должны быть сделана по аналогии с RegularPolygon::Vertices
        //
        [[nodiscard]] constexpr Point2D::PointsContainer Vertices(size_t N = 30) const noexcept {

            return views::iota(0, (int)N) | views::transform([&](auto idx) {
                       const double angle = 2 * std::numbers::pi * idx / N;
                       return Point2D{center_p.x + radius * std::cos(angle), center_p.y + radius * std::sin(angle)};
                   }) |
                   rng::to<Point2D::PointsContainer>();
        }

        [[nodiscard]] constexpr Lines2DDyn Lines(size_t N = 100) const noexcept {
            Point2D::PointsContainer verticies = Vertices(N);
            Lines2DDyn lines = {verticies};
            lines.PushBack(lines.Front());
            return lines;
        }
    };

    class Polygon {
    public:
        constexpr Polygon(std::initializer_list<Point2D> list) : points_(list) {
            CreateBoundingBox();
        }
        constexpr Polygon(Point2D::PointsContainer&& points) : points_(points) {
            CreateBoundingBox();
        }

        [[nodiscard]] constexpr double Height() const noexcept {
            auto [min_y, max_y] = rng::minmax_element(points_, {}, &Point2D::y);
            return std::abs(max_y - min_y);
        }

        [[nodiscard]] constexpr Point2D Center() const noexcept {
            double sum_x = rng::fold_left(points_ | views::transform(&Point2D::x), 0.0, std::plus{});
            double sum_y = rng::fold_left(points_ | views::transform(&Point2D::y), 0.0, std::plus{});

            return {sum_x / points_.size(), sum_y / points_.size()};
        }

        [[nodiscard]] constexpr BoundingBox BoundBox() const noexcept { return bounding_box_; }

        [[nodiscard]] const Point2D::PointsContainer &Vertices() const noexcept { return points_; }

        [[nodiscard]] constexpr Lines2DDyn Lines() const noexcept {
            Lines2DDyn lines = {points_};
            lines.PushBack(lines.Front());
            return lines;
        }

    private:
        void CreateBoundingBox(){
            auto [min_x, max_x] = rng::minmax_element(points_, {}, &Point2D::x);
            auto [min_y, max_y] = rng::minmax_element(points_, {}, &Point2D::y);
            bounding_box_ = {min_x->x, min_y->y, max_x->x, max_y->y};
        };
        Point2D::PointsContainer points_;
        BoundingBox bounding_box_;
    };

    using Shape = std::variant<Line, Triangle, Rectangle, RegularPolygon, Circle, Polygon>;

    enum class GeometryError { Unsupported, NoIntersection, InvalidInput, DegenrateCase, InsufficientPoints };

    template <typename T>
    using GeometryResult = std::expected<T, GeometryError>;

}  // namespace geometry

template <>
struct std::formatter<geometry::Point2D> {
    constexpr auto parse(std::format_parse_context &ctx)  { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Point2D &p, FormatContext &ctx) const {
        return format_to(ctx.out(), "({:.2f}, {:.2f})", p.x, p.y);
    }
};
template <>
struct std::formatter<std::vector<geometry::Point2D>> {
    bool use_new_line = false;

    constexpr auto parse(std::format_parse_context &ctx) {
        auto it = ctx.begin();

        std::string_view fmt{ctx.begin(), ctx.end()};
        constexpr std::string_view specifier = "new_line";

        if (fmt.starts_with(specifier)) {
            use_new_line = true;
            std::advance(it, specifier.size());
        }

        return it;
    }

    template <typename FormatContext>
    auto format(const std::vector<geometry::Point2D> &v, FormatContext &ctx) const {

        std::string_view delimiter = use_new_line ? ",\n\t " : ", ";

        std::format_to(ctx.out(), "Points: [");

        for (const auto &[i, point] : views::enumerate(v)) {
            if (std::cmp_equal(i, v.size() - 1)) {
                // если это последняя итерация
                // то разделитель не выводим
                delimiter = {};
            }
            std::format_to(ctx.out(), "{{{:.2f}:{:.2f}}}{}", point.x, point.y, delimiter);
        }

        std::format_to(ctx.out(), "]");

        return ctx.out();
    }
};

template <>
struct std::formatter<geometry::Line> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Line &l, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "Line({}, {})", l.start, l.end);
    }
};

template <>
struct std::formatter<geometry::Circle> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Circle &c, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "Circle(center={}, r={:.2f})", c.center_p, c.radius);
    }
};

template <>
struct std::formatter<geometry::Rectangle> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Rectangle &r, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "Rectangle(bottom_left={}, w={:.2f}, h={:.2f})", r.bottom_left, r.width,
                              r.height);
    }
};

template <>
struct std::formatter<geometry::RegularPolygon> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::RegularPolygon &p, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "RegularPolygon(center={}, r={:.2f}, sides={})", p.center_p, p.radius,
                              p.sides);
    }
};
template <>
struct std::formatter<geometry::Triangle> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Triangle &t, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "Triangle({}, {}, {})", t.a, t.b, t.c);
    }
};
template <>
struct std::formatter<geometry::Polygon> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Polygon &poly, FormatContext &ctx) const {
        auto out = ctx.out();
        out = std::format_to(out, "Polygon[{} points]: [", poly.Vertices().size());

        for (const auto &p : poly.Vertices()) {
            out = std::format_to(out, "{} ", p);
        }

        return std::format_to(out, "]");
    }
};

template <>
struct std::formatter<geometry::Shape> {
    constexpr auto parse(std::format_parse_context &ctx) {
        return ctx.begin();
    }

    template <typename FormatContext>
    auto format(const geometry::Shape &shape, FormatContext &ctx) const {
        return std::visit([&](const auto &s) { return std::format_to(ctx.out(), "{}", s); }, shape);
    }
};
