#define TINYOBJLOADER_IMPLEMENTATION // define this in only *one* .cc

#include "Renderer.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Sphere.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <chrono>

#include "libs/tiny_obj_loader/tiny_obj_loader.h"
#include "libs/pugixml/pugixml.hpp"

#define STB_IMAGE_IMPLEMENTATION
#include "libs/stb_image/stb_image.h"
// #define STB_IMAGE_WRITE_IMPLEMENTATION
// #include "libs/stb_image/stb_image_write.h"


// In the main function of the program, we create the scene (create objects and
// lights) as well as set the options for the render (image width and height,
// maximum recursion depth, field-of-view, etc.). We then call the render
// function().

int main(int argc, char **argv)
{   
    int spp = 16;
    std::string model_base_dir = "../models/";
    std::string model_name = "veach-mis";
    std::string sampling_mode = "NEE";
    if(argc != 1){
        model_name = std::string(argv[1]);
        spp = atoi(argv[2]);
        sampling_mode = std::string(argv[3]);
    }

    // xml loader 
    std::string input_xml = model_base_dir + model_name + "/" + model_name + ".xml";
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(input_xml.c_str());
    std::cout << "Load result: " << result.description() << std::endl;
    int width = doc.child("camera").attribute("width").as_int();
    int height = doc.child("camera").attribute("height").as_int();
    double fov = doc.child("camera").attribute("fovy").as_double();
    Vector3f eye = Vector3f();
    eye.x = doc.child("camera").child("eye").attribute("x").as_float();
    eye.y = doc.child("camera").child("eye").attribute("y").as_float();
    eye.z = doc.child("camera").child("eye").attribute("z").as_float();
    Vector3f lookat = Vector3f();
    lookat.x = doc.child("camera").child("lookat").attribute("x").as_float();
    lookat.y = doc.child("camera").child("lookat").attribute("y").as_float();
    lookat.z = doc.child("camera").child("lookat").attribute("z").as_float();
    Vector3f up = Vector3f();
    up.x = doc.child("camera").child("up").attribute("x").as_float();
    up.y = doc.child("camera").child("up").attribute("y").as_float();
    up.z = doc.child("camera").child("up").attribute("z").as_float();
    // print camera info
    // std::cout << "camera info: " << std::endl;
    // std::cout << eye << std::endl;
    // std::cout << lookat << std::endl;
    // std::cout << up << std::endl;
    // scene initialization with h, w, eye, lookat, up, fov
    Scene scene(width, height);
    scene.SetCamera(eye, lookat, up, fov);
    scene.setSpp(spp);
    scene.setModelName(model_name);
    scene.setSamplingMode(sampling_mode);


    // prepare <light materials> for object initialization
    std::map<std::string, Vector3f> lights_radiance_map;
    for (pugi::xml_node light = doc.child("light"); light; light = light.next_sibling("light"))
    {
        std::vector<float>tokens;
        std::string radiance_string = light.attribute("radiance").as_string();
        std::string w="";
        for(auto i : radiance_string){
            if(i ==',') tokens.push_back(stof(w)), w ="";
            else if(i != ' ') w+=i;
        }
        tokens.push_back(stof(w));
        lights_radiance_map[light.attribute("mtlname").as_string()] = Vector3f(tokens[0], tokens[1], tokens[2]);
        std::cout << "light:" << light.attribute("mtlname").value() << "'s radiance is: "<< lights_radiance_map[light.attribute("mtlname").as_string()] << std::endl;
    }

    // obj loader
    std::string input_obj = model_base_dir + model_name + "/" + model_name + ".obj";
    tinyobj::ObjReaderConfig reader_config;
    reader_config.mtl_search_path = model_base_dir + model_name + "/"; // Path to material files
    reader_config.vertex_color = false;
    tinyobj::ObjReader reader;

    if (!reader.ParseFromFile(input_obj, reader_config))
    {
        if (!reader.Error().empty())
        {
            std::cerr << "TinyObjReader: " << reader.Error();
        }
        exit(1);
    }
    if (!reader.Warning().empty())
    {
        std::cout << "TinyObjReader: " << reader.Warning();
    }
    auto &attrib = reader.GetAttrib();
    auto &shapes = reader.GetShapes();
    auto &materials = reader.GetMaterials();
    std::cout << "obj has " << shapes.size() << " shapes \n";

    std::vector<MeshTriangle *>Meshes; 

    int image_test_cnt = 0;
    for (size_t s = 0; s < shapes.size(); s++)
    {
        // printf("mesh: %d is %s, has %d faces\n", s, shapes[s].name.c_str(), shapes[s].mesh.num_face_vertices.size());
        MeshTriangle *mesh = new MeshTriangle;
        Material *mt = new Material();
        mesh->m = mt;

        // one shape use only one material 
        int mid = shapes[s].mesh.material_ids[0];
        Vector3f Kd = Vector3f (static_cast<const float>(materials[mid].diffuse[0]),
        static_cast<const float>(materials[mid].diffuse[1]),
        static_cast<const float>(materials[mid].diffuse[2]));
        Vector3f Ks = Vector3f (static_cast<const float>(materials[mid].specular[0]),
        static_cast<const float>(materials[mid].specular[1]),
        static_cast<const float>(materials[mid].specular[2]));
        Vector3f Tr = Vector3f (static_cast<const float>(materials[mid].transmittance[0]),
        static_cast<const float>(materials[mid].transmittance[1]),
        static_cast<const float>(materials[mid].transmittance[2]));
        float Ns = static_cast<const double>(materials[mid].shininess);
        float Ni = static_cast<const double>(materials[mid].ior);
        mt->Kd = Kd; mt->Kd_avg = (Kd.x + Kd.y + Kd.z)/3.0f;
        mt->Ks = Ks; mt->Ks_coe = 1 - std::max( Kd.x , std::max(Kd.y, Kd.z)); mt->Ks_avg = (Ks.x + Ks.y + Ks.z)/3.0f;
        mt->ior = Ni;
        mt->specularExponent = Ns;
        mt->Tr = Tr;
        if(Ks.x !=0 || Ks.y!=0 || Ks.z!=0) {mt->m_type = PHONG;}
        if(materials[mid].diffuse_texname != ""){
            printf("shape %d 's material has a map_Kd: %s\n", s, materials[mid].diffuse_texname.c_str());
            int w, h, n;
            unsigned char* tex = stbi_load((model_base_dir + model_name + "/" + materials[mid].diffuse_texname).c_str(), &w, &h, &n, 3);
            printf("texture's width is %d, height is %d, channels is %d \n", w, h, n);
            mt->loadTex(tex, w, h, n);

            // for(int i=0; i<w/2;i++){
            //     for(int j=0;j<h/2;j++){
            //         tex[n*w*j + i*n+ 0] = 255;
            //         tex[n*w*j + i*n + 1] = 0;
            //         tex[n*w*j + i*n + 2] = 0;
            //     }
            // }
            // stbi_write_jpg(("stbi_test_"+std::to_string(image_test_cnt)+".jpg").c_str(), w, h, n, tex, w);
            // image_test_cnt++;
        }
        // std::cout <<"shape's material's Kd: " <<  Kd << std::endl;

        if(lights_radiance_map.count(materials[mid].name)){
            mt->m_emission =  lights_radiance_map[materials[mid].name];
        }

        size_t index_offset = 0;
        Vector3f min_vert = Vector3f{std::numeric_limits<float>::infinity(),
                                     std::numeric_limits<float>::infinity(),
                                     std::numeric_limits<float>::infinity()};
        Vector3f max_vert = Vector3f{-std::numeric_limits<float>::infinity(),
                                     -std::numeric_limits<float>::infinity(),
                                     -std::numeric_limits<float>::infinity()};

        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++)
        {
            std::array<Vector3f, 3> face_vertices;
            std::array<Vector3f, 3> normals;
            std::array<Vector2f, 3> texcoords;
            for (size_t v = 0; v < 3; v++)
            {
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                Vector3f vert = Vector3f(static_cast<const float>(attrib.vertices[3 * size_t(idx.vertex_index) + 0]),
                                         static_cast<const float>(attrib.vertices[3 * size_t(idx.vertex_index) + 1]),
                                         static_cast<const float>(attrib.vertices[3 * size_t(idx.vertex_index) + 2]));
                face_vertices[v] = vert;
                min_vert = Vector3f(std::min(min_vert.x, vert.x),
                                    std::min(min_vert.y, vert.y),
                                    std::min(min_vert.z, vert.z));
                max_vert = Vector3f(std::max(max_vert.x, vert.x),
                                    std::max(max_vert.y, vert.y),
                                    std::max(max_vert.z, vert.z));

                if (idx.normal_index >= 0)
                {
                    normals[v] = Vector3f(static_cast<const float>(attrib.normals[3 * size_t(idx.normal_index) + 0]),
                                      static_cast<const float>(attrib.normals[3 * size_t(idx.normal_index) + 1]),
                                      static_cast<const float>(attrib.normals[3 * size_t(idx.normal_index) + 2]));
                    // normals[v] = normals[v].normalized();
                }
                if(idx.texcoord_index >= 0)
                {
                    texcoords[v] = Vector2f(static_cast<const float>(attrib.texcoords[2 * size_t(idx.texcoord_index) + 0]),
                                            static_cast<const float>(attrib.texcoords[2 * size_t(idx.texcoord_index) + 1]));
                    // if((texcoords[v].x>1||texcoords[v].x<0) && mt->useTex)
                    //     printf("reading error! reading shape %d, vt.u=%f\n", s, texcoords[v].x);
                    // if( (texcoords[v].y>1||texcoords[v].y<0) && mt->useTex)
                    //     printf("reading error! reading shape %d, vt.v=%f\n", s, texcoords[v].y);
                }
            }
            index_offset += 3;

            mesh->triangles.emplace_back(face_vertices[0], face_vertices[1], face_vertices[2], 
                                        texcoords[0], texcoords[1], texcoords[2], 
                                        normals[0], normals[1], normals[2], mt);
        }
        mesh->bounding_box = Bounds3(min_vert, max_vert);
        std::vector<Object*> ptrs;
        for (auto& tri : mesh->triangles){
            ptrs.push_back(&tri);
            mesh->area += tri.area;
        }
        mesh->bvh = new BVHAccel(ptrs);
        Meshes.emplace_back(mesh);
    }


    // add meshes to scenes
    for(int i=0;i<Meshes.size();i++){
        printf("Mesh: %d has %d triangles, Kd=%f %f %f, emission=%f %f %f\n",
        i, Meshes[i]->triangles.size(), Meshes[i]->m->Kd.x, Meshes[i]->m->Kd.y, Meshes[i]->m->Kd.z,Meshes[i]->m->getEmission().x,Meshes[i]->m->getEmission().y,Meshes[i]->m->getEmission().z);
        if(Meshes[i]->m->ior==1.5f)
            Meshes[i]->m->Tr = Vector3f(0.8f, 1.0f, 0.95f);
        scene.Add(Meshes[i]);
    }


    // Material *red = new Material(DIFFUSE, Vector3f(0.0f));
    // red->Kd = Vector3f(0.63f, 0.065f, 0.05f);
    // Material *green = new Material(DIFFUSE, Vector3f(0.0f));
    // green->Kd = Vector3f(0.14f, 0.45f, 0.091f);
    // Material *white = new Material(DIFFUSE, Vector3f(0.0f));
    // white->Kd = Vector3f(0.725f, 0.71f, 0.68f);
    // Material *light = new Material(DIFFUSE, (8.0f * Vector3f(0.747f + 0.058f, 0.747f + 0.258f, 0.747f) + 15.6f * Vector3f(0.740f + 0.287f, 0.740f + 0.160f, 0.740f) + 18.4f * Vector3f(0.737f + 0.642f, 0.737f + 0.159f, 0.737f)));
    // light->Kd = Vector3f(0.65f);

    // MeshTriangle floor("../models/cornellbox/floor.obj", white);
    // MeshTriangle shortbox("../models/cornellbox/shortbox.obj", white);
    // MeshTriangle tallbox("../models/cornellbox/tallbox.obj", white);
    // MeshTriangle left("../models/cornellbox/left.obj", red);
    // MeshTriangle right("../models/cornellbox/right.obj", green);
    // MeshTriangle light_("../models/cornellbox/light.obj", light);

    // scene.Add(&floor);
    // scene.Add(&shortbox);
    // scene.Add(&tallbox);
    // scene.Add(&left);
    // scene.Add(&right);
    // scene.Add(&light_);


    scene.buildBVH();
    // scene.sumRadiance();
    scene.sumLightsArea();

    Renderer r;

    auto start = std::chrono::system_clock::now();
    r.Render(scene);
    auto stop = std::chrono::system_clock::now();

    std::cout << "Render complete: \n";
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds\n";

    return 0;
}
