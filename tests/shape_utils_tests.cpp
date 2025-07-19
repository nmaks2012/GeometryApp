#include "geometry.hpp"
#include "queries.hpp"
#include "shape_utils.hpp"
#include <gtest/gtest.h>

using namespace geometry;
using namespace geometry::utils;
using namespace geometry::queries;

// ##### Тесты для FindAllCollisions #####
class FindAllCollisionsTest : public ::testing::Test {
protected:
    Circle circle1{{0.0, 0.0}, 1.0}; 
    Circle circle2{{1.5, 0.0}, 1.0};    // Пересекается с circle1
    Circle circle3{{10.0, 10.0}, 1.0};  // Не пересекается
    Line line1{{0.0, 0.0}, {2.0, 2.0}};
    Line line2{{0.0, 2.0}, {2.0, 0.0}};      // Пересекается с line1 
    Line line3{{10.0, 10.0}, {12.0, 12.0}};  // Не пересекается
    Rectangle rect1{{0.0, 0.0}, 2.0, 2.0};
    Rectangle rect2{{1.0, 1.0}, 2.0, 2.0};    // Пересекается с rect1
    Rectangle rect3{{10.0, 10.0}, 2.0, 2.0};  // Не пересекается
};

TEST_F(FindAllCollisionsTest, EmptyInput) {
    std::vector<Shape> shapes;
    auto collisions = FindAllCollisions(shapes);
    EXPECT_TRUE(collisions.empty());
}

TEST_F(FindAllCollisionsTest, SingleShapeNoCollision) {
    std::vector<Shape> shapes = {circle1};
    auto collisions = FindAllCollisions(shapes);
    EXPECT_TRUE(collisions.empty());
}

TEST_F(FindAllCollisionsTest, TwoCollidingCircles) {
    std::vector<Shape> shapes = {circle1, circle2};
    auto collisions = FindAllCollisions(shapes);
    ASSERT_EQ(collisions.size(), 1);
    EXPECT_TRUE(BoundingBoxesOverlap(collisions[0].first, collisions[0].second));
}

TEST_F(FindAllCollisionsTest, TwoNonCollidingCircles) {
    std::vector<Shape> shapes = {circle1, circle3};
    auto collisions = FindAllCollisions(shapes);
    EXPECT_TRUE(collisions.empty());
}

TEST_F(FindAllCollisionsTest, CollidingLines) {
    std::vector<Shape> shapes = {line1, line2};
    auto collisions = FindAllCollisions(shapes);
    ASSERT_EQ(collisions.size(), 1);
    EXPECT_TRUE(BoundingBoxesOverlap(collisions[0].first, collisions[0].second));
}

TEST_F(FindAllCollisionsTest, MixedShapesWithCollisions) {
    std::vector<Shape> shapes = {circle1, line1, rect1};
    auto collisions = FindAllCollisions(shapes);
    // Ожидаемые коллизии: (circle1, line1), (circle1, rect1), (line1, rect1)
    ASSERT_EQ(collisions.size(), 3);
}

TEST_F(FindAllCollisionsTest, MultipleCollisions) {
    std::vector<Shape> shapes = {circle1, circle2, rect1, rect2};
    auto collisions = FindAllCollisions(shapes);
    // Ожидаемые коллизии: (circle1, circle2), (rect1, rect2), (circle1, rect1),
    // (circle1, rect2), (circle2, rect1), (circle2, rect2)
    ASSERT_EQ(collisions.size(), 6);
}

TEST_F(FindAllCollisionsTest, SelfCollisionNotReported) {
    std::vector<Shape> shapes = {circle1, circle1};  // Две одинаковые фигуры
    auto collisions = FindAllCollisions(shapes);
    // Проверяем, что нет пар с одинаковыми индексами
    bool has_self_collision =
        rng::any_of(collisions, [](const std::pair<Shape, Shape> &pair) { return &pair.first == &pair.second; });
    EXPECT_FALSE(has_self_collision);
}

TEST_F(FindAllCollisionsTest, LargeNumberOfShapes) {
    std::vector<Shape> shapes;
    for (int i = 0; i < 100; ++i) {
        shapes.emplace_back(Circle{{static_cast<double>(i), 0.0}, 1.0});
    }
    auto collisions = FindAllCollisions(shapes);
    // Каждая окружность пересекается с соседями (i, i+1)
    // Проверяем что есть хотя бы минимальное ожидаемое количество коллизий
    EXPECT_GT(collisions.size(), 50);
}

// ##### Тесты для FindHighestShape #####
class FindHighestShapeTest : public ::testing::Test {
protected:
    void SetUp() override {
        circle_low = Circle{{0.0, 0.0}, 1.0};      // Высота 2.0 (диаметр)
        rect_medium = Rectangle{{0.0, 0.0}, 3.0, 4.0}; // Высота 4.0
        triangle_high = Triangle{{0.0, 0.0}, {3.0, 0.0}, {1.5, 5.0}}; // Высота 5.0
        line_zero = Line{{0.0, 0.0}, {1.0, 0.0}};   // Высота 0.0
    }

    Shape circle_low;
    Shape rect_medium;
    Shape triangle_high;
    Shape line_zero;
};

TEST_F(FindHighestShapeTest, EmptyInput) {
    std::vector<Shape> shapes;
    auto result = FindHighestShape(shapes);
    EXPECT_FALSE(result.has_value());
}

TEST_F(FindHighestShapeTest, SingleShape) {
    std::vector<Shape> shapes = {rect_medium};
    auto result = FindHighestShape(shapes);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, 0u);
}

TEST_F(FindHighestShapeTest, MultipleShapes) {
    std::vector<Shape> shapes = {circle_low, rect_medium, triangle_high};
    auto result = FindHighestShape(shapes);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, 2u);  // triangle_high должен быть самым высоким
}

TEST_F(FindHighestShapeTest, MultipleShapesSameHeight) {
    std::vector<Shape> shapes = {rect_medium, Rectangle{{0.0, 0.0}, 2.0, 4.0}};
    auto result = FindHighestShape(shapes);
    ASSERT_TRUE(result.has_value());
    // Должен вернуть первый из равных по высоте
    EXPECT_EQ(*result, 0u);
}

TEST_F(FindHighestShapeTest, ShapesWithZeroHeight) {
    std::vector<Shape> shapes = {line_zero, circle_low};
    auto result = FindHighestShape(shapes);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, 1u);  // circle_low выше чем line_zero
}

TEST_F(FindHighestShapeTest, AllShapesSameHeight) {
    std::vector<Shape> shapes = {Circle{{0.0, 0.0}, 1.0}, Circle{{1.0, 0.0}, 1.0}};
    auto result = FindHighestShape(shapes);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, 0u);  // Должен вернуть первый элемент
}

TEST_F(FindHighestShapeTest, MixedShapes) {
    std::vector<Shape> shapes = {line_zero, rect_medium, circle_low, triangle_high};
    auto result = FindHighestShape(shapes);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, 3u);  // triangle_high должен быть самым высоким
}