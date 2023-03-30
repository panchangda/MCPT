//
// Created by Göksu Güvendiren on 2019-05-14.
//

#pragma once

#include <vector>
#include "Vector.hpp"
#include "Object.hpp"
#include "Light.hpp"
#include "AreaLight.hpp"
#include "BVH.hpp"
#include "Ray.hpp"


class Scene
{
public:
    // setting up options
    int width = 1280;
    int height = 960;
    Vector3f eye = Vector3f(278, 273, -800);
    Vector3f lookat = Vector3f(0, 0, -1);
    Vector3f up = Vector3f(0, 1, 0);
    double fov = 40;
    Vector3f backgroundColor = Vector3f(0.235294, 0.67451, 0.843137);
    int maxDepth = 16;
    int spp = 16;
    std::string model_name = "veach-mis";
    std::string sampling_mode = "NEE";


    float RussianRoulette = 0.8;
    float radiance_sum = 0.f;
    float lights_area_sum = 0.f;
    int lights_sum = 0;

    Scene(int w, int h) : width(w), height(h)
    {}

    void setSamplingMode(std::string _m){sampling_mode = _m;}
    void setModelName(std::string _n){model_name = _n;}
    void setSpp(int _spp){spp=_spp;}
    void SetCamera(Vector3f _eye, Vector3f _lookat, Vector3f _up, double _fov){ eye=_eye; lookat=_lookat; _up = up; fov = _fov; }
    void Add(Object *object) { objects.push_back(object); }
    void Add(std::unique_ptr<Light> light) { lights.push_back(std::move(light)); }

    const std::vector<Object*>& get_objects() const { return objects; }
    const std::vector<std::unique_ptr<Light> >&  get_lights() const { return lights; }
    Intersection intersect(const Ray& ray) const;
    BVHAccel *bvh;
    void buildBVH();
    void buildScene();
    void sumRadiance();
    void sumLightsArea();

    Vector3f Direct_Illumination(const Ray &ray, Intersection &inter, float& pdf) const;
    Vector3f castRay(const Ray &ray, int depth) const;
    Vector3f castRay_BRDF(const Ray &ray, int depth) const;
    Vector3f castRay_NEE(const Ray &ray, int depth) const;
    Vector3f castRay_MIS(const Ray &ray, int depth) const;
    
    void sampleLight(Intersection &pos, float &pdf) const;
    // void Scene::sampleManyLights(Intersection &pos, float &pdf) const;
    bool trace(const Ray &ray, const std::vector<Object*> &objects, float &tNear, uint32_t &index, Object **hitObject);
    std::tuple<Vector3f, Vector3f> HandleAreaLight(const AreaLight &light, const Vector3f &hitPoint, const Vector3f &N,
                                                   const Vector3f &shadowPointOrig,
                                                   const std::vector<Object *> &objects, uint32_t &index,
                                                   const Vector3f &dir, float specularExponent);

    // creating the scene (adding objects and lights)
    std::vector<Object* > objects;
    std::vector<std::unique_ptr<Light> > lights;

    // Compute reflection direction
    Vector3f reflect(const Vector3f &I, const Vector3f &N) const
    {
        return I - 2 * dotProduct(I, N) * N;
    }



// Compute refraction direction using Snell's law
//
// We need to handle with care the two possible situations:
//
//    - When the ray is inside the object
//
//    - When the ray is outside.
//
// If the ray is outside, you need to make cosi positive cosi = -N.I
//
// If the ray is inside, you need to invert the refractive indices and negate the normal N
    Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        Vector3f n = N;
        if (cosi < 0) { cosi = -cosi; } else { std::swap(etai, etat); n= -N; }
        float eta = etai / etat;
        float k = 1 - eta * eta * (1 - cosi * cosi);
        return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
    }



    // Compute Fresnel equation
//
// \param I is the incident view direction
//
// \param N is the normal at the intersection point
//
// \param ior is the material refractive index
//
// \param[out] kr is the amount of light reflected
    void fresnel(const Vector3f &I, const Vector3f &N, const float &ior, float &kr) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        if (cosi > 0) {  std::swap(etai, etat); }
        // Compute sini using Snell's law
        float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
        // Total internal reflection
        if (sint >= 1) {
            kr = 1;
        }
        else {
            float cost = sqrtf(std::max(0.f, 1 - sint * sint));
            cosi = fabsf(cosi);
            float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
            float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
            kr = (Rs * Rs + Rp * Rp) / 2;
        }
        // As a consequence of the conservation of energy, transmittance is given by:
        // kt = 1 - kr;
    }
};