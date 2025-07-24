#include "geometry.hpp"
#include <algorithm>
#include <format>
#include <gtest/gtest.h>
#include <print>
#include <ranges>

using namespace geometry;

// ####### Тесты для formatter std::vector<geometry::Point2D> #######
TEST(FormatterTest, EmptyPointsContainer) {
    Point2D::PointsContainer points = {};
    auto fmt = std::format("{}", points);
    EXPECT_EQ(fmt, "Points: []");
}

TEST(FormatterTest, OneLinePointsContainer) {
    Point2D::PointsContainer points = {{0, 0}, {1, 1}};
    auto fmt = std::format("{}", points);
    EXPECT_EQ(fmt, "Points: [{0.00:0.00}, {1.00:1.00}]");
}

TEST(FormatterTest, ManyLinesPointsContainer) {
    Point2D::PointsContainer points = {{0, 0}, {1, 1}, {2, 2}};
    auto fmt = std::format("{:new_line}", points);
    EXPECT_EQ(fmt, "Points: [{0.00:0.00},\n\t {1.00:1.00},\n\t {2.00:2.00}]");
}

// ####### Тесты для Line #######
class LineTests : public ::testing::Test {
protected:
    Line horizontal{{0, 0}, {5, 0}};
    Line vertical{{0, 0}, {0, 5}};
    Line diagonal{{0, 0}, {3, 4}};
    Line diagonal_offset{{1, 1}, {4, 5}};
    Line degenerate{{0, 0}, {0, 0}};
};

TEST_F(LineTests, Length) {
    EXPECT_DOUBLE_EQ(horizontal.Length(), 5.0);
    EXPECT_DOUBLE_EQ(vertical.Length(), 5.0);
    EXPECT_DOUBLE_EQ(diagonal.Length(), 5.0);
    EXPECT_DOUBLE_EQ(diagonal_offset.Length(), 5.0);
    EXPECT_DOUBLE_EQ(degenerate.Length(), 0.0);
}

TEST_F(LineTests, Direction) {

    EXPECT_DOUBLE_EQ(horizontal.Direction().x, -1.0);
    EXPECT_DOUBLE_EQ(horizontal.Direction().y, 0.0);

    EXPECT_DOUBLE_EQ(vertical.Direction().x, 0.0);
    EXPECT_DOUBLE_EQ(vertical.Direction().y, -1.0);

    EXPECT_DOUBLE_EQ(diagonal.Direction().x, -0.6);
    EXPECT_DOUBLE_EQ(diagonal.Direction().y, -0.8);

    EXPECT_DOUBLE_EQ(degenerate.Direction().x, 0.0);
    EXPECT_DOUBLE_EQ(degenerate.Direction().y, 0.0);
}

TEST_F(LineTests, BoundingBox) {

    BoundingBox bbox_horizontal = horizontal.BoundBox();
    EXPECT_DOUBLE_EQ(bbox_horizontal.min_x, 0.0);
    EXPECT_DOUBLE_EQ(bbox_horizontal.min_y, 0.0);
    EXPECT_DOUBLE_EQ(bbox_horizontal.max_x, 5.0);
    EXPECT_DOUBLE_EQ(bbox_horizontal.max_y, 0.0);

    BoundingBox bbox_vertical = vertical.BoundBox();
    EXPECT_DOUBLE_EQ(bbox_vertical.min_x, 0.0);
    EXPECT_DOUBLE_EQ(bbox_vertical.min_y, 0.0);
    EXPECT_DOUBLE_EQ(bbox_vertical.max_x, 0.0);
    EXPECT_DOUBLE_EQ(bbox_vertical.max_y, 5.0);

    BoundingBox bbox_diag_offset = diagonal_offset.BoundBox();
    EXPECT_DOUBLE_EQ(bbox_diag_offset.min_x, 1.0);
    EXPECT_DOUBLE_EQ(bbox_diag_offset.min_y, 1.0);
    EXPECT_DOUBLE_EQ(bbox_diag_offset.max_x, 4.0);
    EXPECT_DOUBLE_EQ(bbox_diag_offset.max_y, 5.0);

    BoundingBox bbox_degenerate = degenerate.BoundBox();
    EXPECT_DOUBLE_EQ(bbox_degenerate.min_x, 0.0);
    EXPECT_DOUBLE_EQ(bbox_degenerate.min_y, 0.0);
    EXPECT_DOUBLE_EQ(bbox_degenerate.max_x, 0.0);
    EXPECT_DOUBLE_EQ(bbox_degenerate.max_y, 0.0);
}

TEST_F(LineTests, Height) {
    EXPECT_DOUBLE_EQ(horizontal.Height(), 0.0);
    EXPECT_DOUBLE_EQ(vertical.Height(), 5.0);
    EXPECT_DOUBLE_EQ(diagonal.Height(), 4.0);
    EXPECT_DOUBLE_EQ(diagonal_offset.Height(), 4.0);
    EXPECT_DOUBLE_EQ(degenerate.Height(), 0.0);
}

TEST_F(LineTests, Center) {

    auto [h_cx, h_cy] = horizontal.Center();
    EXPECT_DOUBLE_EQ(h_cx, 2.5);
    EXPECT_DOUBLE_EQ(h_cy, 0.0);

    auto [v_cx, v_cy] = vertical.Center();
    EXPECT_DOUBLE_EQ(v_cx, 0.0);
    EXPECT_DOUBLE_EQ(v_cy, 2.5);

    auto [d_cx, d_cy] = diagonal.Center();
    EXPECT_DOUBLE_EQ(d_cx, 1.5);
    EXPECT_DOUBLE_EQ(d_cy, 2.0);

    auto [deg_cx, deg_cy] = degenerate.Center();
    EXPECT_DOUBLE_EQ(deg_cx, 0.0);
    EXPECT_DOUBLE_EQ(deg_cy, 0.0);
}

TEST_F(LineTests, Vertices) {

    auto h_vertices = horizontal.Vertices();
    EXPECT_EQ(h_vertices[0].x, 0);
    EXPECT_EQ(h_vertices[0].y, 0);
    EXPECT_EQ(h_vertices[1].x, 5);
    EXPECT_EQ(h_vertices[1].y, 0);

    auto d_vertices = diagonal.Vertices();
    EXPECT_EQ(d_vertices[0].x, 0);
    EXPECT_EQ(d_vertices[0].y, 0);
    EXPECT_EQ(d_vertices[1].x, 3);
    EXPECT_EQ(d_vertices[1].y, 4);

    auto deg_vertices = degenerate.Vertices();
    EXPECT_EQ(deg_vertices[0].x, 0);
    EXPECT_EQ(deg_vertices[0].y, 0);
    EXPECT_EQ(deg_vertices[1].x, 0);
    EXPECT_EQ(deg_vertices[1].y, 0);
}

TEST_F(LineTests, Lines) {

    auto [h_x, h_y] = horizontal.Lines();
    EXPECT_EQ(h_x[0], 0);
    EXPECT_EQ(h_y[0], 0);
    EXPECT_EQ(h_x[1], 5);
    EXPECT_EQ(h_y[1], 0);

    auto [d_x, d_y] = diagonal.Lines();
    EXPECT_EQ(d_x[0], 0);
    EXPECT_EQ(d_y[0], 0);
    EXPECT_EQ(d_x[1], 3);
    EXPECT_EQ(d_y[1], 4);

    auto [deg_x, deg_y] = degenerate.Lines();
    EXPECT_EQ(deg_x[0], 0);
    EXPECT_EQ(deg_y[0], 0);
    EXPECT_EQ(deg_x[1], 0);
    EXPECT_EQ(deg_y[1], 0);
}

// ####### Тесты для Triangle #######
class TriangleTests : public ::testing::Test {
protected:
    Triangle triangle{{0, 0}, {3, 0}, {0, 4}};
};

TEST_F(TriangleTests, Vertices) {
    auto vertices = triangle.Vertices();
    ASSERT_EQ(vertices.size(), 3);
    EXPECT_EQ(vertices[0], Point2D(0, 0));
    EXPECT_EQ(vertices[1], Point2D(3, 0));
    EXPECT_EQ(vertices[2], Point2D(0, 4));
}

TEST_F(TriangleTests, Lines) {

    auto lines = triangle.Lines();
    ASSERT_EQ(lines.x.size(), 4);
    ASSERT_EQ(lines.y.size(), 4);
    EXPECT_EQ(lines.x[0], 0);
    EXPECT_EQ(lines.x[1], 3);
    EXPECT_EQ(lines.x[2], 0);
    EXPECT_EQ(lines.x[3], 0);
    EXPECT_EQ(lines.y[0], 0);
    EXPECT_EQ(lines.y[1], 0);
    EXPECT_EQ(lines.y[2], 4);
    EXPECT_EQ(lines.y[3], 0);
}

TEST_F(TriangleTests, Area) { EXPECT_DOUBLE_EQ(triangle.Area(), 6.0); }

TEST_F(TriangleTests, Height) { EXPECT_DOUBLE_EQ(triangle.Height(), 4.0); }

TEST_F(TriangleTests, Center) {
    Point2D center = triangle.Center();

    EXPECT_DOUBLE_EQ(center.x, (0 + 3 + 0) / 3.0);
    EXPECT_DOUBLE_EQ(center.y, (0 + 0 + 4) / 3.0);
}

TEST_F(TriangleTests, BoundBox) {
    auto bbox = triangle.BoundBox();

    EXPECT_EQ(bbox.min_x, 0);
    EXPECT_EQ(bbox.min_y, 0);
    EXPECT_EQ(bbox.max_x, 3);
    EXPECT_EQ(bbox.max_y, 4);
}

// ####### Тесты для Rectangle #######
class RectangleTests : public ::testing::Test {
protected:
    Rectangle rect{{1, 1}, 4, 3};
};

TEST_F(RectangleTests, Vertices) {
    auto vertices = rect.Vertices();
    ASSERT_EQ(vertices.size(), 4);
    EXPECT_EQ(vertices[0], Point2D(1, 1));
    EXPECT_EQ(vertices[1], Point2D(5, 1));
    EXPECT_EQ(vertices[2], Point2D(1, 4));
    EXPECT_EQ(vertices[3], Point2D(5, 4));
}

TEST_F(RectangleTests, Lines) {
    auto lines = rect.Lines();
    ASSERT_EQ(lines.x.size(), 5);
    ASSERT_EQ(lines.y.size(), 5);
    EXPECT_EQ(lines.x[0], 1);
    EXPECT_EQ(lines.x[1], 1);
    EXPECT_EQ(lines.x[2], 5);
    EXPECT_EQ(lines.x[3], 5);
    EXPECT_EQ(lines.x[4], 1);
    EXPECT_EQ(lines.y[0], 1);
    EXPECT_EQ(lines.y[1], 4);
    EXPECT_EQ(lines.y[2], 4);
    EXPECT_EQ(lines.y[3], 1);
    EXPECT_EQ(lines.y[4], 1);
}

TEST_F(RectangleTests, Area) { EXPECT_DOUBLE_EQ(rect.Area(), 12.0); }

TEST_F(RectangleTests, Height) { EXPECT_DOUBLE_EQ(rect.Height(), 3.0); }

TEST_F(RectangleTests, Center) {
    Point2D center = rect.Center();
    EXPECT_DOUBLE_EQ(center.x, 3.0);
    EXPECT_DOUBLE_EQ(center.y, 2.5);
}

TEST_F(RectangleTests, BoundBox) {
    auto bbox = rect.BoundBox();
    EXPECT_EQ(bbox.min_x, 1);
    EXPECT_EQ(bbox.min_y, 1);
    EXPECT_EQ(bbox.max_x, 5);
    EXPECT_EQ(bbox.max_y, 4);
}

// ####### Тесты для RegularPolygon #######
class RegularPolygonTests : public ::testing::Test {
protected:
    RegularPolygon hexagon{{0, 0}, 2.0, 6};
};

TEST_F(RegularPolygonTests, Vertices) {
    auto vertices = hexagon.Vertices();
    ASSERT_EQ(vertices.size(), 6);

    EXPECT_DOUBLE_EQ(vertices[0].x, 2.0);
    EXPECT_DOUBLE_EQ(vertices[0].y, 0.0);

    EXPECT_DOUBLE_EQ(vertices[1].x, 1.0);
    EXPECT_DOUBLE_EQ(vertices[1].y, std::sqrt(3.0));

    EXPECT_DOUBLE_EQ(vertices[2].x, -1.0);
    EXPECT_DOUBLE_EQ(vertices[2].y, std::sqrt(3.0));
}

TEST_F(RegularPolygonTests, Lines) {
    auto lines = hexagon.Lines();
    ASSERT_EQ(lines.x.size(), 7);
    ASSERT_EQ(lines.y.size(), 7);

    EXPECT_DOUBLE_EQ(lines.x[0], 2.0);
    EXPECT_DOUBLE_EQ(lines.y[0], 0.0);

    EXPECT_DOUBLE_EQ(lines.x[6], 2.0);
    EXPECT_DOUBLE_EQ(lines.y[6], 0.0);
}

TEST_F(RegularPolygonTests, Height) {
    auto height = hexagon.Height();
    EXPECT_DOUBLE_EQ(height, 3.4641016151377548);
}

TEST_F(RegularPolygonTests, Center) {
    Point2D center = hexagon.Center();
    EXPECT_DOUBLE_EQ(center.x, 0.0);
    EXPECT_DOUBLE_EQ(center.y, 0.0);
}

TEST_F(RegularPolygonTests, BoundBox) {
    auto bbox = hexagon.BoundBox();
    EXPECT_DOUBLE_EQ(bbox.min_x, -2.0);
    EXPECT_DOUBLE_EQ(bbox.min_y, -1.7320508075688772);
    EXPECT_DOUBLE_EQ(bbox.max_x, 2.0);
    EXPECT_DOUBLE_EQ(bbox.max_y, 1.7320508075688772);
}

// ####### Тесты для Circle #######
class CircleTests : public ::testing::Test {
protected:
    Circle circle{{2, 3}, 5.0};
};

TEST_F(CircleTests, Vertices) {
    auto vertices = circle.Vertices();
    ASSERT_EQ(vertices.size(), 30);
    EXPECT_DOUBLE_EQ(vertices[0].x, 7.0);
    EXPECT_DOUBLE_EQ(vertices[0].y, 3.0);
}

TEST_F(CircleTests, Lines) {
    auto lines = circle.Lines();
    ASSERT_EQ(lines.x.size(), 101);
    ASSERT_EQ(lines.y.size(), 101);

    EXPECT_DOUBLE_EQ(lines.x[0], 7.0);
    EXPECT_DOUBLE_EQ(lines.y[0], 3.0);

    EXPECT_DOUBLE_EQ(lines.x[100], 7.0);
    EXPECT_DOUBLE_EQ(lines.y[100], 3.0);
}

TEST_F(CircleTests, Height) { EXPECT_DOUBLE_EQ(circle.Height(), 10.0); }

TEST_F(CircleTests, Center) {
    Point2D center = circle.Center();
    EXPECT_DOUBLE_EQ(center.x, 2.0);
    EXPECT_DOUBLE_EQ(center.y, 3.0);
}

TEST_F(CircleTests, BoundBox) {
    auto bbox = circle.BoundBox();
    EXPECT_DOUBLE_EQ(bbox.min_x, -3.0);
    EXPECT_DOUBLE_EQ(bbox.min_y, -2.0);
    EXPECT_DOUBLE_EQ(bbox.max_x, 7.0);
    EXPECT_DOUBLE_EQ(bbox.max_y, 8.0);
}

// ####### Тесты для Polygon #######
class PolygonTests : public ::testing::Test {

protected:
    Polygon polygon{{{0, 0}, {2, 0}, {3, 2}, {1, 3}, {-1, 1}}};
};

TEST_F(PolygonTests, Vertices) {
    auto vertices = polygon.Vertices();
    Point2D::PointsContainer expected{{0, 0}, {2, 0}, {3, 2}, {1, 3}, {-1, 1}};
    ASSERT_EQ(vertices.size(), expected.size());
    EXPECT_TRUE(rng::all_of(views::zip(vertices, expected),
                            [](const auto &tp) { return std::get<0>(tp) == std::get<1>(tp); }));
}

TEST_F(PolygonTests, Lines) { 
    auto lines = polygon.Lines(); 
    Point2D::PointsContainer expected{{0, 0}, {2, 0}, {3, 2}, {1, 3}, {-1, 1}, {0, 0}};
    ASSERT_EQ(lines.x.size(), expected.size());
    ASSERT_EQ(lines.y.size(), expected.size());
    EXPECT_TRUE(rng::all_of(views::zip(lines.x, lines.y, expected),
                            [](const auto &tp) { 
                                auto [x, y, point] = tp;
                                return x == point.x && y == point.y; 
                            }));
}

TEST_F(PolygonTests, Height) { EXPECT_DOUBLE_EQ(polygon.Height(), 3.0); }

TEST_F(PolygonTests, Center) {
    Point2D center = polygon.Center();
    
    EXPECT_DOUBLE_EQ(center.x, (0 + 2 + 3 + 1 - 1) / 5.0);
    EXPECT_DOUBLE_EQ(center.y, (0 + 0 + 2 + 3 + 1) / 5.0);
}

TEST_F(PolygonTests, BoundBox) {
    auto bbox = polygon.BoundBox();

    EXPECT_DOUBLE_EQ(bbox.min_x, -1.0);
    EXPECT_DOUBLE_EQ(bbox.min_y, 0.0);
    EXPECT_DOUBLE_EQ(bbox.max_x, 3.0);
    EXPECT_DOUBLE_EQ(bbox.max_y, 3.0);
}

// ####### Тесты для BoundingBox #######
class BoundingBoxTests : public ::testing::Test {
protected:
    BoundingBox mainBox{0, 0, 5, 5};
};

TEST_F(BoundingBoxTests, OverlapsWithOtherBoundingBox) {
    BoundingBox otherBox(3, 3, 6, 6);
    EXPECT_TRUE(mainBox.Overlaps(otherBox));
    
    BoundingBox nonOverlappingBox(6, 6, 8, 8);
    EXPECT_FALSE(mainBox.Overlaps(nonOverlappingBox));
}

TEST_F(BoundingBoxTests, OverlapsWithCircle) {

    Circle innerCircle({2, 2}, 1.0);
    EXPECT_TRUE(mainBox.Overlaps(innerCircle.BoundBox()));
    
    Circle overlappingCircle({5, 5}, 2.0);
    EXPECT_TRUE(mainBox.Overlaps(overlappingCircle.BoundBox()));
    
    Circle outerCircle({7, 7}, 1.0);
    EXPECT_FALSE(mainBox.Overlaps(outerCircle.BoundBox()));
}

TEST_F(BoundingBoxTests, OverlapsWithTriangle) {

    Triangle innerTriangle({1, 1}, {2, 1}, {1, 2});
    EXPECT_TRUE(mainBox.Overlaps(innerTriangle.BoundBox()));
    
    Triangle overlappingTriangle({4, 4}, {6, 4}, {4, 6});
    EXPECT_TRUE(mainBox.Overlaps(overlappingTriangle.BoundBox()));
    
    Triangle outerTriangle({6, 6}, {7, 6}, {6, 7});
    EXPECT_FALSE(mainBox.Overlaps(outerTriangle.BoundBox()));
}

TEST_F(BoundingBoxTests, OverlapsWithRectangle) {

    Rectangle innerRect({1, 1}, 2, 2);
    EXPECT_TRUE(mainBox.Overlaps(innerRect.BoundBox()));
    
    Rectangle overlappingRect({4, 4}, 3, 3);
    EXPECT_TRUE(mainBox.Overlaps(overlappingRect.BoundBox()));
    
    Rectangle outerRect({6, 6}, 2, 2);
    EXPECT_FALSE(mainBox.Overlaps(outerRect.BoundBox()));
}

TEST_F(BoundingBoxTests, OverlapsWithRegularPolygon) {

    RegularPolygon innerHexagon({2, 2}, 1.0, 6);
    EXPECT_TRUE(mainBox.Overlaps(innerHexagon.BoundBox()));
    
    RegularPolygon overlappingHexagon({5, 5}, 2.0, 6);
    EXPECT_TRUE(mainBox.Overlaps(overlappingHexagon.BoundBox()));
    
    RegularPolygon outerHexagon({7, 7}, 1.0, 6);
    EXPECT_FALSE(mainBox.Overlaps(outerHexagon.BoundBox()));
}

TEST_F(BoundingBoxTests, OverlapsWithPolygon) {

    Polygon innerPolygon({{1,1}, {2,1}, {2,2}, {1,2}});
    EXPECT_TRUE(mainBox.Overlaps(innerPolygon.BoundBox()));
    
    Polygon overlappingPolygon({{4,4}, {6,4}, {6,6}, {4,6}});
    EXPECT_TRUE(mainBox.Overlaps(overlappingPolygon.BoundBox()));
    
    Polygon outerPolygon({{6,6}, {7,6}, {7,7}, {6,7}});
    EXPECT_FALSE(mainBox.Overlaps(outerPolygon.BoundBox()));
}

TEST_F(BoundingBoxTests, EdgeCases) {

    BoundingBox touchingBox(5, 0, 7, 5);
    EXPECT_TRUE(mainBox.Overlaps(touchingBox));
    
    BoundingBox zeroBox(1, 1, 1, 1);
    EXPECT_TRUE(mainBox.Overlaps(zeroBox));
    
    BoundingBox negativeBox(-2, -2, 1, 1);
    EXPECT_TRUE(mainBox.Overlaps(negativeBox));
}