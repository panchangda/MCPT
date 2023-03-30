//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include <thread>
#include <mutex>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "libs/stb_image/stb_image_write.h"

std::mutex mtx;
int progress = 0;

inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos = scene.eye;
    Vector3f c = (scene.eye - scene.lookat).normalized(); 
    Vector3f a = crossProduct(scene.up, c).normalized();
    Vector3f b = crossProduct(c, a).normalized();
    // std::cout << "view transformation is:" << std::endl;
    // std::cout << a << std::endl;
    // std::cout << b << std::endl;
    // std::cout << c << std::endl;
    int m = 0;

    // change the spp value to change sample ammount
    int spp = scene.spp;
    std::cout << "SPP: " << spp << "\n";
    int num_threads = 32;
    std::thread th[num_threads];
    int thread_height = scene.height/num_threads;
    auto renderRows = [&](uint32_t start_height, uint32_t end_height) {
        for (uint32_t j = start_height; j < end_height; ++j) {
            for (uint32_t i = 0; i < scene.width; ++i) {
                // generate primary ray direction
                float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                        imageAspectRatio * scale;
                float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

                float z = -1;
                Vector3f ray_world_coord = Vector3f(x, y, z);
                // transform rays from world coord to camera coord
                x = a.x*ray_world_coord.x + b.x * ray_world_coord.y + c.x*ray_world_coord.z;
                y = a.y*ray_world_coord.x + b.y * ray_world_coord.y + c.y*ray_world_coord.z;
                z = a.z*ray_world_coord.x + b.z * ray_world_coord.y + c.z*ray_world_coord.z;

                Vector3f dir = normalize(Vector3f(x, y, z));
                for (int k = 0; k < spp; k++){
                    if(scene.sampling_mode == "NEE")
                        framebuffer[(int)(j*scene.width+i)] += scene.castRay_NEE(Ray(eye_pos, dir), 0) / spp;  
                    else if(scene.sampling_mode == "BRDF")
                        framebuffer[(int)(j*scene.width+i)] += scene.castRay_BRDF(Ray(eye_pos, dir), 0) / spp; 
                    else if(scene.sampling_mode == "MIS")
                        framebuffer[(int)(j*scene.width+i)] += scene.castRay_MIS(Ray(eye_pos, dir), 0) / spp;
                        
                }
            }
            mtx.lock();
            progress++;
            UpdateProgress(progress / (float)scene.height);
            mtx.unlock();
        }};
    for (int t = 0; t < num_threads; ++t) {
        th[t] = std::thread(renderRows, t*thread_height, (t+1)*thread_height);
    }
    for (int t = 0; t < num_threads; ++t) {
        th[t].join();
    }

    UpdateProgress(1.f);

    // save framebuffer to file
    // FILE* fp = fopen((scene.model_name+".ppm").c_str(), "wb");
    // (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    // for (auto i = 0; i < scene.height * scene.width; ++i) {
    //     static unsigned char color[3];
    //     color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
    //     color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
    //     color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
    //     fwrite(color, 1, 3, fp);
    // }
    // fclose(fp);    

    // write to png
    unsigned char *result_image_buffer = (unsigned char *) malloc(scene.width*scene.height* 3);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        result_image_buffer[3*i + 0] = color[0];
        result_image_buffer[3*i + 1] = color[1];
        result_image_buffer[3*i + 2] = color[2];
    }
    std::string result_png_file = scene.model_name+".png";
    stbi_write_png(result_png_file.c_str(), scene.width, scene.height, 3, result_image_buffer, 0);
}
