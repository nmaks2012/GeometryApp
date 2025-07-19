#include "convex_hull.hpp"
#include "geometry.hpp"
#include "intersections.hpp"
#include "queries.hpp"
#include "shape_utils.hpp"
#include "triangulation.hpp"
#include "visualization.hpp"

#include <algorithm>
#include <array>
#include <optional>
#include <print>
#include <ranges>
#include <span>
#include <utility>

using namespace geometry;

namespace rng = std::ranges;
namespace views = std::ranges::views;

void PrintAllIntersections(const Shape &shape, const std::span<Shape> others) {
    std::println("\n=== Intersections ===");

    /*
     * Используйте ranges чтобы оставить только фигуры,
     * поддерживающие возможность находить пересечения между собой
     *
     * Затем примените монадический интерфейс для обработки результатов:
     *     - Пересечение найдено в точке A между фигурами B и C
     *     - Фигуры B и C не пересекаются
     */
    // clang-format off
    if(others.empty()){
        std::println("Ошибка: не переданы фигуры для сравнения");
        return;
    }
    auto filtered_shapes = others | views::filter([&](const auto& other_shape) {
                                        return queries::DistanceBetweenShapes(shape, other_shape).has_value();
                                    });

    rng::for_each(filtered_shapes, [&](const auto& other_shape){
        intersections::GetIntersectPoint(shape, other_shape)
            .and_then([&](const auto& point) {
                std::println("Пересечение найдено в точке {} между фигурами {} и {}", point, shape, other_shape);
                return std::optional<std::monostate>{std::in_place};
            })
            .or_else([&] {
                std::println("Фигуры {} и {} не пересекаются", shape, other_shape);
                return std::optional<std::monostate>{std::in_place};
            });
    });
    // clang-format on
}

void PrintDistancesFromPointToShapes(Point2D p, const std::span<Shape> shapes) {
    std::println("\n=== Distance from Point Test ===");
    std::println("Testing point: {} ", p);

    /*
     * Используйте ranges чтобы выбрать любые 5 фигур из списка.
     * Затем найдите расстояния от заданной точки до всех выбранных фигур.
     * Выведите результат в формате "Расстояние от точки P до фигуры S равно D"
     */
    rng::for_each(shapes | views::take(5), [&](const auto &shape) {
        const double distance = queries::DistanceToPoint(shape, p);
        std::println("Расстояние от точки {} до фигуры {} равно {:.2f}", p, shape, distance);
    });
}

void PerformShapeAnalysis(const std::span<Shape> shapes) {
    std::println("\n=== Shape Analysis ===");

    /*
     * Используйте ranges и созданные классы чтобы:
     *     - Найти все пересечения между фигурами используя метод Bounding Box
     *     - Найти самую высокую фигуру (чья высота наибольшая)
     *     - Вывести расстояние между любыми двумя фигурами, которые поддерживают данную функциональность
     */
    // Найти все пересечения между фигурами используя метод Bounding Box
    auto collisions = utils::FindAllCollisions(shapes);
    if (collisions.empty()) {
        std::println("Коллизии не найдены.");
    } else {
        std::println("Найдено {} коллизий:", collisions.size());
        rng::for_each(collisions, [](const auto &collision) {
            std::println("  - Коллизия между => {} && {}", collision.first, collision.second);
        });
    }

    // Найти самую высокую фигуру (чья высота наибольшая)
    if (auto highest_idx_opt = utils::FindHighestShape(shapes)) {
        const auto &highest_shape = shapes[*highest_idx_opt];
        std::println("Самая высокая фигура {} с индексом {}", highest_shape, *highest_idx_opt);
        std::println("  - Ее высота == {:.2f}", queries::GetHeight(highest_shape));
    } else {
        std::println("Не удалось определить самую высокую фигуру.");
    }

    // Вывести расстояние между любыми двумя фигурами, которые поддерживают данную функциональность
    auto indices = views::iota(0u, shapes.size());
    auto two_shapes = views::cartesian_product(indices, indices) | views::filter([&](const auto &pair) {
                          const auto [i, j] = pair;
                          return i < j && queries::DistanceBetweenShapes(shapes[i], shapes[j]).has_value();
                      }) |
                      views::take(1) | views::transform([&](const auto &pair) {
                          auto [i, j] = pair;
                          return std::make_pair(shapes[i], shapes[j]);
                      });
    if (two_shapes.empty()) {
        std::println("Не удалось определить фигуры между которыми можно найти расстояние");
    } else {
        std::println("Найдены 2 фигуры: ");
        std::println("  - Фигура 1: {}", two_shapes.front().first);
        std::println("  - Фигура 2: {}", two_shapes.front().second);
        std::println("  - Расстояние: {:.2f}",
                     *queries::DistanceBetweenShapes(two_shapes.front().first, two_shapes.front().second));
    }
}

void PerformExtraShapeAnalysis(std::span<const Shape> shapes) {
    std::println("\n=== Shape Extra Analysis ===");

    /*
     * Используйте ranges и созданные классы чтобы:
     *     - Вывести 3 любые фигуры, которые находятся выше 50.0
     *     - Вывести фигуры с наименьшей и с наибольшей высотами
     */
    // Вывести 3 любые фигуры, которые находятся выше 50.0
    auto high_shapes = shapes | views::enumerate |
                       views::filter([](const auto &pair) { return queries::GetHeight(std::get<1>(pair)) > 50.0; }) |
                       views::take(3);
    if (high_shapes.empty()) {
        std::println("фигуры выше 50.0 не найдены");
    } else {
        std::println("Фигуры выше 50.0:");
        rng::for_each(high_shapes, [](const auto &pair) {
            std::println("  - Фигура {} с индексом {}, выстота {}", std::get<1>(pair), std::get<0>(pair),
                         queries::GetHeight(std::get<1>(pair)));
        });
    }

    // Вывести фигуры с наименьшей и с наибольшей высотами
    // - Находим фигуру с минимальной высотой
    auto min_height_shape_it = rng::min_element(shapes, {}, [](const auto &s) { return queries::GetHeight(s); });
    // - Находим фигуру с максимальной высотой
    auto max_height_shape_it = rng::max_element(shapes, {}, [](const auto &s) { return queries::GetHeight(s); });
    if (min_height_shape_it != shapes.end() && max_height_shape_it != shapes.end()) {
        std::println("Фигуры с минимальной и максимальной высотой:");
        std::println("  - Минимальная высота: {} ({:.2f})", *min_height_shape_it,
                     queries::GetHeight(*min_height_shape_it));
        std::println("  - Максимальная высота: {} ({:.2f})", *max_height_shape_it,
                     queries::GetHeight(*max_height_shape_it));
    } else {
        std::println("  - Не удалось определить фигуры с минимальной и максимальной высотой.");
    }
}

int main() {
    utils::ShapeGenerator generator(-50.0, 50.0, 5.0, 25.0);
    std::vector<Shape> shapes = generator.GenerateShapes(50);

    std::println("Generated {} random shapes", shapes.size());

    // Выведите индекс каждой фигуры и её высоту

    //
    // Вызываем разработанные функции
    //
    PrintAllIntersections(shapes[0], shapes);

    PrintDistancesFromPointToShapes(Point2D{10.0, 10.0}, shapes);

    PerformShapeAnalysis(shapes);

    PerformExtraShapeAnalysis(shapes);

    //
    // Рисуем все фигуры
    //
    // Важно: после изучения графика - нажмите Enter чтобы продолжить выполнение и построить 2ой график
    //
    geometry::visualization::Draw(shapes);

    // Находим выпуклую оболочку
    auto convex_hull_result = convex_hull::GrahamScan(shapes);

    if (!convex_hull_result) {
        std::println("Ошибка при построении выпуклой оболочки");
        return 1;
    }

    // Создаем полигон из выпуклой оболочки
    shapes.emplace_back(Polygon(std::move(*convex_hull_result)));

    // Рисуем все фигуры с выпуклой оболочкой
    geometry::visualization::Draw(std::span(shapes));

    //
    // после изучения графика - нажмите Enter чтобы продолжить выполнение и построить 3ий график
    //

    {
        std::vector<Point2D> points = {{0, 0}, {10, 0}, {5, 8}, {15, 5}, {2, 12}};

        // Выполняем триангуляцию Делоне
        auto triangulation_result = triangulation::DelaunayTriangulation(points);
        if (!triangulation_result) {
            std::println("Ошибка при выполнении триангуляции Делоне");
            return 1;
        }
        visualization::Draw(*triangulation_result);

    }

    return 0;
}