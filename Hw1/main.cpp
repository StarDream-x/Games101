#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    rotation_angle = rotation_angle / 180.0 * acos(-1);
    float cs = std::cos(rotation_angle), sn = std::sin(rotation_angle);
    model(0,0) = cs; model(0,1) = -sn;
    model(1,0) = sn; model(1,1) = cs;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    Eigen::Matrix4f pTo,orth;
    float n = zNear, f = zFar;
    pTo << n, 0.0f, 0.0f, 0.0f, 0.0f, n, 0.0f, 0.0f, 0.0f, 0.0f, n+f, -(n * f), 0.0f, 0.0f, 1.0f, 0.0f;
    eye_fov = eye_fov / 360 * acos(-1);
    float t = n * std::tan(eye_fov);
    float l = t * aspect_ratio;
    orth << 1/l, 0.0f, 0.0f, 0.0f, 0.0f, 1/t, 0.0f, 0.0f, 0.0f, 0.0f, 2 / (f - n), -(f + n) / (f - n), 0.0f, 0.0f, 0.0f, 1.0f;
    projection = orth * pTo;
    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle){
    angle = angle / 180 * acos(-1);
    Eigen::Matrix3f rt1,rt2,rt3;
    rt1 = Eigen::Matrix3f::Identity();
    rt1 = std::cos(angle) * rt1;
    rt2 = axis * axis.transpose();
    rt2 = (1.0f - std::cos(angle)) * rt2;
    rt3 << 0.0f, -axis(2), axis(1), axis(2), 0.0f, -axis(0), -axis(1), axis(0), 0.0f;
    rt3 = std::sin(angle) * rt3;
    rt1 = rt1 + rt2 + rt3;
    Eigen::Matrix4f res;
    res << rt1(0,0), rt1(0,1), rt1(0,2), 0.0f, rt1(1,0), rt1(1,1), rt1(1,2), 0.0f, rt1(2,0), rt1(2,1), rt1(2,2), 0.0f, 0.0f, 0.0f, 0.0f ,1.0f;
    return res;


    /*
    * Below is a wrong way to solve this
    */

    // Eigen::Vector4f axi;
    // Eigen::RowVector4f raxi;
    // axi << axis.x(), axis.y(), axis.z(), 0.0f;
    // raxi << axis.x(), axis.y(), axis.z(), 0.0f;
    // Eigen::Matrix4f I = Eigen::Matrix4f::Identity();
    // Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
    // Eigen::Matrix4f N = Eigen::Matrix4f::Identity();
    // N << 0.0f, -axis.z(), axis.y(), 0.0f,
    //      axis.z(), 0.0f, -axis.x(), 0.0f,
    //      -axis.y(), axis.x(), 0.0f, 0.0f,
    //      0.0f, 0.0f, 0.0f, 1.0f;

    // rotation = cos(angle)*I+(1-cos(angle))*axi*raxi+sin(angle)*N;
    // // rotation(3,3) = 1.0f; //Modify to right
    // return rotation;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation(Vector3f(1.0f, 1.0f, 1.0f), angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
