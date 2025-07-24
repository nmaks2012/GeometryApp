#include "geometry.hpp"
#include "queries.hpp"
#include <gtest/gtest.h>

using namespace geometry;
using namespace queries;

// ####### Тесты для DistanceToPoint #######
class DistanceToPointTest : public ::testing::Test {
protected:
    Line line{{0, 0}, {4, 0}};
    Circle circle{{0, 0}, 2};
    Triangle triangle{{0, 0}, {4, 0}, {2, 3}};
    Rectangle rectangle{{1, 1}, 3, 2};
    RegularPolygon regular_poly{{0, 0}, 2, 6};
    Polygon polygon{{0, 0}, {2, 0}, {2, 2}, {0, 2}};
};

TEST_F(DistanceToPointTest, LineDistance) {
    // Точка над линией
    EXPECT_DOUBLE_EQ(DistanceToPoint(line, {2, 3}), 3.0);
    // Точка под линией
    EXPECT_DOUBLE_EQ(DistanceToPoint(line, {3, -4}), 4.0);
    // Точка на линии
    EXPECT_DOUBLE_EQ(DistanceToPoint(line, {1, 0}), 0.0);
    // Точка за пределами сегмента (но проекция попадает на сегмент)
    EXPECT_DOUBLE_EQ(DistanceToPoint(line, {5, 5}), std::sqrt(26));
    // Точка за пределами сегмента (проекция не попадает)
    EXPECT_DOUBLE_EQ(DistanceToPoint(line, {-2, 3}), std::hypot(2, 3));
}

// ===== Тесты для окружности =====
TEST_F(DistanceToPointTest, CircleDistance_Outside) {
    EXPECT_DOUBLE_EQ(DistanceToPoint(circle, {3, 0}), 1.0);   // Справа
    EXPECT_DOUBLE_EQ(DistanceToPoint(circle, {0, 3}), 1.0);   // Сверху
    EXPECT_DOUBLE_EQ(DistanceToPoint(circle, {-3, 0}), 1.0);  // Слева
}

TEST_F(DistanceToPointTest, CircleDistance_OnEdge) {
    EXPECT_DOUBLE_EQ(DistanceToPoint(circle, {2, 0}), 0.0);
    EXPECT_DOUBLE_EQ(DistanceToPoint(circle, {0, 2}), 0.0);
}

TEST_F(DistanceToPointTest, CircleDistance_Inside) {
    EXPECT_DOUBLE_EQ(DistanceToPoint(circle, {1, 0}), 0);  // Внутри, на радиусе
    EXPECT_DOUBLE_EQ(DistanceToPoint(circle, {1, 1}), 0);  // Внутри, диагональ
}

// ===== Тесты для треугольника =====
TEST_F(DistanceToPointTest, TriangleDistance_Outside) {
    // Под основанием
    EXPECT_DOUBLE_EQ(DistanceToPoint(triangle, {2, -1}), 1.0);
    // Над вершиной
    EXPECT_DOUBLE_EQ(DistanceToPoint(triangle, {2, 4}), 1.0);
    // Сбоку от ребра
    EXPECT_DOUBLE_EQ(DistanceToPoint(triangle, {5, 1}), 1.3867504905630728);
}

TEST_F(DistanceToPointTest, TriangleDistance_OnEdge) {
    EXPECT_DOUBLE_EQ(DistanceToPoint(triangle, {2, 0}), 0.0);    // На основании
    EXPECT_DOUBLE_EQ(DistanceToPoint(triangle, {1, 1.5}), 0.0);  // На боковой стороне
}

TEST_F(DistanceToPointTest, TriangleDistance_Inside) {
    EXPECT_DOUBLE_EQ(DistanceToPoint(triangle, {2, 1}), 0.0);      // В центре
    EXPECT_DOUBLE_EQ(DistanceToPoint(triangle, {1.5, 0.5}), 0.0);  // Внутри
}

// ===== Тесты для прямоугольника =====
TEST_F(DistanceToPointTest, RectangleDistance_Outside) {
    // Справа
    EXPECT_DOUBLE_EQ(DistanceToPoint(rectangle, {5, 2}), 1.0);
    // Сверху
    EXPECT_DOUBLE_EQ(DistanceToPoint(rectangle, {2.5, 4}), 1.0);
    // В углу
    EXPECT_DOUBLE_EQ(DistanceToPoint(rectangle, {-1, -1}), std::hypot(2, 2));
}

TEST_F(DistanceToPointTest, RectangleDistance_OnEdge) {
    EXPECT_DOUBLE_EQ(DistanceToPoint(rectangle, {2.5, 1}), 0.0);  // На нижней грани
    EXPECT_DOUBLE_EQ(DistanceToPoint(rectangle, {1, 3}), 0.0);    // На боковой грани
}

TEST_F(DistanceToPointTest, RectangleDistance_Inside) {
    EXPECT_DOUBLE_EQ(DistanceToPoint(rectangle, {2, 2}), 0.0);      // В центре
    EXPECT_DOUBLE_EQ(DistanceToPoint(rectangle, {1.5, 1.5}), 0.0);  // Внутри
}

// ===== Тесты для правильного шестиугольника =====
TEST_F(DistanceToPointTest, RegularPolygonDistance_Outside) {
    // На оси X за пределами
    EXPECT_DOUBLE_EQ(DistanceToPoint(regular_poly, {3, 0}), 1.0);
    // Между вершинами
    EXPECT_DOUBLE_EQ(DistanceToPoint(regular_poly, {2, 2}), 1.0);
}

TEST_F(DistanceToPointTest, RegularPolygonDistance_OnVertex) {
    EXPECT_DOUBLE_EQ(DistanceToPoint(regular_poly, {2, 0}), 0.0);
}

TEST_F(DistanceToPointTest, RegularPolygonDistance_Inside) {
    EXPECT_DOUBLE_EQ(DistanceToPoint(regular_poly, {1, 0}), 0.0);  // На радиусе
    EXPECT_DOUBLE_EQ(DistanceToPoint(regular_poly, {0, 0}), 0.0);  // В центре
}

// ===== Тесты для произвольного полигона =====
TEST_F(DistanceToPointTest, PolygonDistance_Outside) {
    // Справа от квадрата
    EXPECT_DOUBLE_EQ(DistanceToPoint(polygon, {3, 1}), 1.0);
    // Над квадратом
    EXPECT_DOUBLE_EQ(DistanceToPoint(polygon, {1, 3}), 1.0);
    // В углу
    EXPECT_DOUBLE_EQ(DistanceToPoint(polygon, {-1, -1}), std::hypot(1, 1));
}

TEST_F(DistanceToPointTest, PolygonDistance_OnEdge) {
    EXPECT_DOUBLE_EQ(DistanceToPoint(polygon, {1, 0}), 0.0);  // На нижней грани
    EXPECT_DOUBLE_EQ(DistanceToPoint(polygon, {2, 1}), 0.0);  // На правой грани
}

TEST_F(DistanceToPointTest, PolygonDistance_Inside) {
    EXPECT_DOUBLE_EQ(DistanceToPoint(polygon, {1, 1}), 0.0);      // В центре
    EXPECT_DOUBLE_EQ(DistanceToPoint(polygon, {0.5, 0.5}), 0.0);  // Внутри
}

// ===== Особые случаи =====
TEST_F(DistanceToPointTest, DegenerateLine) {
    Line degenerate{{1, 1}, {1, 1}};  // Вырожденная линия (точка)
    EXPECT_DOUBLE_EQ(DistanceToPoint(degenerate, {4, 5}), 5.0);  // Расстояние до точки
}

TEST_F(DistanceToPointTest, ZeroRadiusCircle) {
    Circle point_circle{{3, 3}, 0};  // Окружность нулевого радиуса
    EXPECT_DOUBLE_EQ(DistanceToPoint(point_circle, {3, 4}), 1.0);
}

// ####### Тесты для GetBoundBox #######
class GetBoundBoxTest : public ::testing::Test {
protected:
    Line line{{1, 1}, {4, 5}};
    Circle circle{{2, 2}, 3};
};

TEST_F(GetBoundBoxTest, LineBoundBox) {
    BoundingBox box = GetBoundBox(line);
    EXPECT_DOUBLE_EQ(box.min_x, 1.0);
    EXPECT_DOUBLE_EQ(box.max_x, 4.0);
}

TEST_F(GetBoundBoxTest, CircleBoundBox) {
    BoundingBox box = GetBoundBox(circle);
    EXPECT_DOUBLE_EQ(box.min_x, -1.0);
    EXPECT_DOUBLE_EQ(box.max_x, 5.0);
}

// ####### Тесты для GetHeight #######
class GetHeightTest : public ::testing::Test {
protected:
    Line line{{0, 0}, {4, 3}};
    Triangle triangle{{0, 0}, {4, 0}, {2, 6}};
};

TEST_F(GetHeightTest, LineHeight) { EXPECT_DOUBLE_EQ(GetHeight(line), 3.0); }

TEST_F(GetHeightTest, TriangleHeight) { EXPECT_DOUBLE_EQ(GetHeight(triangle), 5.0); }

// ####### Тесты для BoundingBoxesOverlap #######
class BoundingBoxesOverlapTest : public ::testing::Test {
protected:
    Line line1{{0, 0}, {4, 0}};
    Line line2{{2, -1}, {2, 1}};
    Circle circle{{3, 0}, 1};
};

TEST_F(BoundingBoxesOverlapTest, LinesOverlap) { EXPECT_TRUE(BoundingBoxesOverlap(line1, line2)); }

TEST_F(BoundingBoxesOverlapTest, LineCircleOverlap) { EXPECT_TRUE(BoundingBoxesOverlap(line1, circle)); }

// ####### Тесты для DistanceBetweenShapes #######
class DistanceBetweenShapesTest : public ::testing::Test {
protected:
    Line line1{{0, 0}, {4, 0}};
    Line line2{{2, 1}, {2, 3}};
    Circle circle1{{0, 0}, 1};
    Circle circle2{{3, 0}, 1};
};

TEST_F(DistanceBetweenShapesTest, LinesDistance) {
    auto dist = DistanceBetweenShapes(line1, line2);
    ASSERT_TRUE(dist.has_value());
    EXPECT_DOUBLE_EQ(dist.value(), 1.0);
}

TEST_F(DistanceBetweenShapesTest, CirclesDistance) {
    auto dist = DistanceBetweenShapes(circle1, circle2);
    ASSERT_TRUE(dist.has_value());
    EXPECT_DOUBLE_EQ(dist.value(), 1.0);
}

TEST_F(DistanceBetweenShapesTest, NoDistanceCase) {
    auto dist = DistanceBetweenShapes(line1, circle1);
    EXPECT_FALSE(dist.has_value());
}

TEST_F(DistanceBetweenShapesTest, IntersectingLines) {
    Line line3{{0, 0}, {2, 2}};
    Line line4{{0, 2}, {2, 0}};
    auto dist = DistanceBetweenShapes(line3, line4);
    EXPECT_TRUE(dist.has_value());
}

TEST_F(DistanceBetweenShapesTest, ParallelLines) {
    Line line3{{0, 1}, {4, 1}};
    auto dist = DistanceBetweenShapes(line1, line3);
    ASSERT_TRUE(dist.has_value());
    EXPECT_DOUBLE_EQ(dist.value(), 1.0);
}

TEST_F(DistanceBetweenShapesTest, CoincidentLines) {
    Line line3{{1, 0}, {3, 0}};
    auto dist = DistanceBetweenShapes(line1, line3);
    EXPECT_DOUBLE_EQ(dist.value(), 0.0);
}

TEST_F(DistanceBetweenShapesTest, TangentCircles) {
    Circle circle3{{2, 0}, 1};
    auto dist = DistanceBetweenShapes(circle1, circle3);
    ASSERT_TRUE(dist.has_value());
    EXPECT_DOUBLE_EQ(dist.value(), 0.0);
}

TEST_F(DistanceBetweenShapesTest, IntersectingCircles) {
    Circle circle3{{1.5, 0}, 1};
    auto dist = DistanceBetweenShapes(circle1, circle3);
    EXPECT_DOUBLE_EQ(dist.value(), 0.0);
}

TEST_F(DistanceBetweenShapesTest, NestedCircles) {
    Circle circle3{{0, 0}, 3};
    auto dist = DistanceBetweenShapes(circle1, circle3);
    EXPECT_DOUBLE_EQ(dist.value(), 0.0);
}

TEST_F(DistanceBetweenShapesTest, PerpendicularLines) {
    Line line3{{1, -1}, {1, 1}};
    auto dist = DistanceBetweenShapes(line1, line3);
    ASSERT_TRUE(dist.has_value());
    EXPECT_DOUBLE_EQ(dist.value(), 1.0);
}

TEST_F(DistanceBetweenShapesTest, ZeroRadiusCircles) {
    Circle point1{{5, 0}, 0};
    Circle point2{{7, 0}, 0};
    auto dist = DistanceBetweenShapes(point1, point2);
    ASSERT_TRUE(dist.has_value());
    EXPECT_DOUBLE_EQ(dist.value(), 2.0);
}

TEST_F(DistanceBetweenShapesTest, LineSegmentInsideOther) {
    Line line3{{1, 0}, {3, 0}};
    auto dist = DistanceBetweenShapes(line1, line3);
    EXPECT_DOUBLE_EQ(dist.value(), 0.0);
}