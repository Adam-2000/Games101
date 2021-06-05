// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f point;
    point << x, y, 0;
    int i;

    for(i = 0; i < 3; i++){
        if(_v[i + 3].cross(point - _v[i]).z() < 0){
            return false;
        }
    }
    return true;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}
static float computeBarycentric2D_z(float x, float y, const Vector3f* t_v, const std::array<Vector4f, 3>& v){
    auto res_tuple = computeBarycentric2D(x, y, t_v);
    auto alpha = std::get<0>(res_tuple) ;
    auto beta = std::get<1>(res_tuple) ;
    auto gamma = std::get<2>(res_tuple) ;
    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    z_interpolated *= w_reciprocal;
    return z_interpolated;
}




void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    int x_l, x_r, y_t, y_b;
    int i, j;
    int x, y;
    int ind;
    int count_SS;
    float samp_x, samp_y;
    float z_interpolated;
    x_l = v[0].x();
    x_r = v[0].x();
    y_t = v[0].y();
    y_b = v[0].y();
    for(i = 0; i < 3; i++){
        std::cout << v[i] << std::endl;
    }
    for(i = 1; i < 3; i++){
        x_l = x_l < v[i].x() ? x_l : (int) v[i].x();
        x_r = x_r > v[i].x() ? x_r : (int) v[i].x();
        y_t = y_t > v[i].y() ? y_t : (int) v[i].y();
        y_b = y_b < v[i].y() ? y_b : (int) v[i].y();
    }
    // iterate through the pixel and find if the current pixel is inside the triangle
    //std::cout << x_l << x_r << std::endl;
    //std::cout << y_b << y_t << std::endl;
    //std::cout << t.v[0] << "  " << t.v[1] << "  " << t.v[2] << "  " << std::endl;
    Eigen::Vector3f vectors_tri[6];
    //std::cout << "Points" << std::endl;
    for(i = 0; i < 3; i++){
        vectors_tri[i] << t.v[i].x(), t.v[i].y(), 0;
        //std::cout << vectors_tri[i] << std::endl;
    }
    //std::cout << "Vectors" << std::endl;
    for(i = 0; i < 3; i++){
        vectors_tri[i + 3] = vectors_tri[(i + 1) % 3] - vectors_tri[i];
        //std::cout << vectors_tri[i + 3] << std::endl;
    }
    for(y = y_b; y <= y_t; y++){
        for(x = x_l; x <= x_r; x++){
            /*
            if(insideTriangle(x + 0.5, y + 0.5, vectors_tri)){
                // If so, use the following code to get the interpolated z value.
                z_interpolated = computeBarycentric2D_z(x + 0.5, y + 0.5, t.v, v);
                //std::cout << z_interpolated << depth_buf[get_index(x, y)] << std::endl;
                if(z_interpolated < depth_buf[get_index(x, y)][0]){
                    depth_buf[get_index(x, y)][0] = -z_interpolated;
                    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
                    frame_buf[get_index(x, y)] = t.getColor();
                    //std::cout << "1" << std::endl;
                }
            }
            //*/
           ///*
           ind = get_index(x, y);
           for(i = 0; i <= 2; i++){
               for(j = 0; j <= 2; j++){
                   samp_x = x + 0.5 * j;
                   samp_y = y + 0.5 * i;
                   if(insideTriangle(samp_x + 0.25, samp_y + 0.25, vectors_tri)){
                        z_interpolated = computeBarycentric2D_z(samp_x + 0.25, samp_y + 0.25, t.v, v);
                        if(z_interpolated < depth_buf[ind][i + j * 2]){
                            depth_buf[ind][i + j * 2] = -z_interpolated;
                            frame_buf_SS[ind * 4 + i + j * 2] = t.getColor();
                        }
                   }
                }
            }
            frame_buf[ind] = (frame_buf_SS[ind * 4] + frame_buf_SS[ind * 4 + 1] + frame_buf_SS[ind * 4 + 2] + frame_buf_SS[ind * 4 + 3]) / 4;
            //*/
           
        }
    }



    
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {   std::array<float, 4> array_4_inf;
        std::fill(array_4_inf.begin(), array_4_inf.end(), std::numeric_limits<float>::infinity());
        std::fill(depth_buf.begin(), depth_buf.end(), std::array<float, 4>(array_4_inf));   
    }
    if ((buff & rst::Buffers::Color_SS) == rst::Buffers::Color_SS)
    {   
        std::fill(frame_buf_SS.begin(), frame_buf_SS.end(), Eigen::Vector3f{0, 0, 0});
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    frame_buf_SS.resize(4 * w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on