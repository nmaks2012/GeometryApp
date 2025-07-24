#include "geometry.hpp"
#include "intersections.hpp"
#include <cstdlib>
#include <gtest/gtest.h>

using namespace geometry;
using namespace geometry::intersections;

// ####### Тесты для проверки пересечений между Line и Line #######
struct LinesIntersectionTests : public ::testing::Test {
    Line horizontal{{0, 0}, {4, 0}};
    Line vertical{{2, -2}, {2, 2}};
    Line diagonal1{{1, 1}, {3, 3}};
    Line diagonal2{{1, 3}, {3, 1}};
    Line parallel{{0, 1}, {4, 1}};
};

// Пересечение под прямым углом
TEST_F(LinesIntersectionTests, PerpendicularIntersection) {
    auto result = GetIntersectPoint(horizontal, vertical);
    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(result.value()[0].x, 2.0);
    EXPECT_DOUBLE_EQ(result.value()[0].y, 0.0);
}

// Пересечение диагоналей
TEST_F(LinesIntersectionTests, DiagonalIntersection) {
    auto result = GetIntersectPoint(diagonal1, diagonal2);
    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(result.value()[0].x, 2.0);
    EXPECT_DOUBLE_EQ(result.value()[0].y, 2.0);
}

// Параллельные отрезки
TEST_F(LinesIntersectionTests, ParallelSegments) {
    auto result = GetIntersectPoint(horizontal, parallel);
    EXPECT_FALSE(result.has_value());
}

// Пересечение на границе
TEST_F(LinesIntersectionTests, BorderIntersection) {
    Line edge_case{{2, 0}, {2, 2}};
    auto result = GetIntersectPoint(horizontal, edge_case);
    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(result.value()[0].x, 2.0);
    EXPECT_DOUBLE_EQ(result.value()[0].y, 0.0);
}

// Отрезки не пересекаются
TEST_F(LinesIntersectionTests, NoIntersection) {
    Line far_segment{{5, 5}, {6, 6}};
    auto result = GetIntersectPoint(horizontal, far_segment);
    EXPECT_FALSE(result.has_value());
}

// Совпадающие отрезки
TEST_F(LinesIntersectionTests, CollinearSegments) {
    Line same_horizontal{{1, 0}, {3, 0}};
    auto result = GetIntersectPoint(horizontal, same_horizontal);
    EXPECT_FALSE(result.has_value());
}

// ####### Тесты для проверки пересечений между Line и Circle #######
struct LineCircleIntersectionTests : public ::testing::Test {

    Circle unit_circle{{0, 0}, 1.0};
    
    Line horizontal{{-2, 0}, {2, 0}};  
    Line vertical{{0, -2}, {0, 2}};       
    Line tangent{{1, -1}, {1, 1}};        
    Line outside{{3, 0}, {4, 0}};        
    Line chord{{-0.5, 0}, {0.5, 0}};      
    Line diagonal{{-1, -1}, {1, 1}};      
    Line point{{0.5, 0.5}, {0.5, 0.5}};   
};

// Пересечение в двух точках
TEST_F(LineCircleIntersectionTests, TwoIntersectionPoints) {
    auto points = GetIntersectPoint(unit_circle, horizontal);
    ASSERT_EQ(points.value().size(), 2);
    EXPECT_DOUBLE_EQ(points.value()[0].x, 1.0);
    EXPECT_DOUBLE_EQ(points.value()[0].y, 0.0);
    EXPECT_DOUBLE_EQ(points.value()[1].x, -1.0);
    EXPECT_DOUBLE_EQ(points.value()[1].y, 0.0);
}

// Касание в одной точке
TEST_F(LineCircleIntersectionTests, TangentIntersection) {
    auto points = GetIntersectPoint(unit_circle, tangent);
    ASSERT_EQ(points.value().size(), 1);
    EXPECT_DOUBLE_EQ(points.value()[0].x, 1.0);
    EXPECT_DOUBLE_EQ(points.value()[0].y, 0.0);
}

// Нет пересечений
TEST_F(LineCircleIntersectionTests, NoIntersection) {
    auto points = GetIntersectPoint(unit_circle, outside);
    EXPECT_FALSE(points.has_value());
}

// Отрезок полностью внутри окружности (хорда)
TEST_F(LineCircleIntersectionTests, ChordInsideCircle) {
    auto points = GetIntersectPoint(unit_circle, chord);
    EXPECT_FALSE(points.has_value());
}

// Диагональное пересечение
TEST_F(LineCircleIntersectionTests, DiagonalIntersection) {
    auto points = GetIntersectPoint(unit_circle, diagonal);
    ASSERT_EQ(points.value().size(), 2);
    EXPECT_NEAR(points.value()[0].x, 0.707107, 1e-5);
    EXPECT_NEAR(points.value()[0].y, 0.707107, 1e-5);
    EXPECT_NEAR(points.value()[1].x, -0.707107, 1e-5);
    EXPECT_NEAR(points.value()[1].y, -0.707107, 1e-5);
}

// ####### Тесты для проверки пересечений между Circle и Circle #######
struct CircleIntersectionTests : public ::testing::Test {

    Circle base_circle{{0, 0}, 2.0}; // Центр (0,0), радиус 2
    
    Circle concentric{{0, 0}, 1.0};
    Circle same_radius{{3, 0}, 2.0}; 
    Circle intersecting{{2, 0}, 2.0}; 
    Circle inside_touch{{0, 1}, 1.0}; 
    Circle separate{{5, 0}, 1.0};
    Circle identical{{0, 0}, 2.0};   
    Circle small_inside{{0, 0}, 0.5};
};

// Пересечение в двух точках
TEST_F(CircleIntersectionTests, TwoIntersectionPoints) {
    auto points = GetIntersectPoint(base_circle, intersecting);
    ASSERT_EQ(points.value().size(), 2);
    EXPECT_DOUBLE_EQ(points.value()[0].x, 1.0);
    EXPECT_DOUBLE_EQ(points.value()[0].y, -std::sqrt(3.0));
    EXPECT_DOUBLE_EQ(points.value()[1].x, 1.0);
    EXPECT_DOUBLE_EQ(points.value()[1].y, std::sqrt(3.0));
}

// Касание извне (1 точка)
TEST_F(CircleIntersectionTests, ExternalTangent) {
    auto points = GetIntersectPoint(base_circle, same_radius);
    ASSERT_EQ(points.value().size(), 2);
    EXPECT_DOUBLE_EQ(points.value()[0].x, 1.5);
    EXPECT_DOUBLE_EQ(points.value()[0].y, -1.3228756555322954);
}

// Нет пересечений (отдельные окружности)
TEST_F(CircleIntersectionTests, NoIntersection) {
    auto points = GetIntersectPoint(base_circle, separate);
    EXPECT_TRUE(!points.has_value());
}

// Внутреннее касание (1 точка)
TEST_F(CircleIntersectionTests, InternalTangent) {
    auto points = GetIntersectPoint(base_circle, inside_touch);
    ASSERT_EQ(points.value().size(), 1);
    EXPECT_DOUBLE_EQ(points.value()[0].x, 0.0);
    EXPECT_DOUBLE_EQ(points.value()[0].y, 2.0);
}

// Концентрические окружности
TEST_F(CircleIntersectionTests, ConcentricCircles) {
    auto points = GetIntersectPoint(base_circle, concentric);
    EXPECT_TRUE(!points.has_value());
}

// ####### Остальные тесты #######

// Неподдерживаемые типы
TEST(UnsupportedShapesTest, PolygonCircle){
    Polygon polygon{{{0, 0}, {2, 0}, {3, 2}, {1, 3}, {-1, 1}}};
    Circle circle{{2, 3}, 5.0};
    auto result = GetIntersectPoint(polygon, circle);
    EXPECT_FALSE(result);
}