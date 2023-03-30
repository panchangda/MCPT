//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH()
{
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}
// void Scene::sumRadiance()
// {
//     Vector3f scene_radiance_sum = Vector3f();
//     int scene_lights_sum = 0;
//     for (int i = 0; i < objects.size(); i++)
//     {
//         if (objects[i]->hasEmit())
//         {
//             scene_radiance_sum += objects[i]->getEmission();
//             scene_lights_sum++;
//         }
//     }
//     radiance_sum = scene_radiance_sum.x + scene_radiance_sum.y + scene_radiance_sum.z;
//     lights_sum = scene_lights_sum;
// }
void Scene::sumLightsArea()
{
    float scene_lights_area_sum = 0.f;
    int scene_lights_sum = 0;
    for (int i = 0; i < objects.size(); i++)
    {
        if (objects[i]->hasEmit())
        {
            scene_lights_area_sum += objects[i]->getArea();
            scene_lights_sum++;
        }
    }
    lights_area_sum = scene_lights_area_sum;
    lights_sum = scene_lights_sum;
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

// void Scene::sampleLight(Intersection &pos, float &pdf) const
// {
//     float emit_area_sum = 0;
//     for (uint32_t k = 0; k < objects.size(); ++k)
//     {
//         if (objects[k]->hasEmit())
//         {
//             emit_area_sum += objects[k]->getArea();
//         }
//     }
//     float p = get_random_float() * emit_area_sum;

//     emit_area_sum = 0;
//     for (uint32_t k = 0; k < objects.size(); ++k)
//     {
//         if (objects[k]->hasEmit())
//         {
//             emit_area_sum += objects[k]->getArea();
//             if (p <= emit_area_sum)
//             {
//                 objects[k]->Sample(pos, pdf);
//                 break;
//             }
//         }
//     }
// }
void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    int selected_light = (int)(get_random_float() * lights_sum);
    selected_light == lights_sum?0:selected_light;
    int tmp_light = 0;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            if(tmp_light == selected_light){
                objects[k]->Sample(pos, pdf);
                break;
            }
            else    tmp_light++;
        }
    }
}

bool Scene::trace(
    const Ray &ray,
    const std::vector<Object *> &objects,
    float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear)
        {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }

    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
/* 
Path Tracing with
    1. Next Event Estimation (direct light + indirect light)
*/
Vector3f Scene::Direct_Illumination(const Ray &ray, Intersection &inter, float &pdf) const
{   

    Vector3f p = inter.coords;
    Vector3f N = inter.normal;
    Vector3f wo = (ray.origin - p).normalized();

    // Backward Light: stop recursive trace
    if(dotProduct(wo, N) < 0){
        return Vector3f();
    }

        Vector3f L_dir;

        // Once Sampling All Lights  
        // for(uint32_t k=0;k<objects.size();k++)
        // {
        //     if(objects[k]->hasEmit())
        //     {
        //         Vector3f colori;
        //             Intersection inter_light;
        //             float pdf_light;
        //             objects[k]->Sample(inter_light, pdf_light);
        //             Vector3f x = inter_light.coords;
        //             Vector3f ws = (x - p).normalized();
        //             Vector3f NN = inter_light.normal;
        //             Vector3f emit = inter_light.emit;
        //             Ray p2x(p, ws);
        //             Intersection middle = intersect(p2x);
        //             float accuracy = 1e-3;
        //             if ((x - p).norm() - accuracy < (middle.coords - p).norm())
        //             {
        //                 L_dir += emit * inter.m->eval(wo, ws, N, inter.diffuse_color) * dotProduct(ws, N) * dotProduct(-ws, NN) / std::pow((x - p).norm(), 2) / pdf_light;
        //                 pdf += pdf_light;
        //             }
        //     }
        // }
        // L_dir = L_dir / lights_sum;

        // Once Sampling Single Light  (randomly choose 1 area light)
        Intersection inter_light;
        float pdf_light;
        sampleLight(inter_light, pdf_light);
        Vector3f x = inter_light.coords;
        Vector3f ws = (x - p).normalized();
        Vector3f NN = inter_light.normal;
        Vector3f emit = inter_light.emit;
        // denoise: remove vertical black lines in cornell-box
        Vector3f newP = dotProduct(p,N) > 0 ? p + N*1e-3 : p - N*1e-3;
        Ray p2x(newP, ws);
        Intersection middle = intersect(p2x);
        float accuracy = 1e-3;
        if ((x - p).norm() - accuracy < (middle.coords - p).norm())
        {
            L_dir = emit * inter.m->BRDF_eval(wo, ws, N, inter.diffuse_color) * dotProduct(ws, N) * dotProduct(-ws, NN) / std::pow((x - p).norm(), 2) / pdf_light;
            pdf = pdf_light;
        }

        // Direct Illumination Estimation
        return L_dir;

}

Vector3f Scene::castRay_NEE(const Ray &ray, int depth) const
{
    if(depth>maxDepth) return Vector3f(0.5f,0.5f,0.5f);

    Intersection inter = intersect(ray);

    if (!inter.happened)
    {
        return Vector3f();
    }
    else if (inter.m->hasEmission())
    {   
        return inter.m->getEmission();
        if (depth == 0)
            return inter.m->getEmission();
        else
            return Vector3f(0, 0, 0);
    }

    Vector3f p = inter.coords;
    Vector3f N = inter.normal;
    Vector3f wo = (ray.origin - p).normalized();

    // Backward Light: stop recursive trace
    if(dotProduct(wo, N) < 0){
        return Vector3f();
    }

        // reflect & refract
    if (inter.m->ior != 1){
        Vector3f reflectionDirection = normalize(reflect(ray.direction, N));
        Vector3f refractionDirection = normalize(refract(ray.direction, N, inter.m->ior));
        Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ? p - N * EPSILON : p + N * EPSILON;
        Vector3f refractionRayOrig = (dotProduct(refractionDirection, N) < 0) ? p - N * EPSILON : p + N * EPSILON;
        Vector3f reflectionColor = castRay_NEE(Ray(reflectionRayOrig, reflectionDirection), depth + 1);
        Vector3f refractionColor = castRay_NEE(Ray(refractionRayOrig, refractionDirection), depth + 1);
        float kr;
        fresnel(ray.direction, N, inter.m->ior, kr);
        return reflectionColor * kr + refractionColor * (1 - kr) * inter.m->Tr;
    }
    // phong reflectance model
   

        Vector3f L_dir, L_indir;
        float pdf_DI = 0;
        L_dir = Direct_Illumination(ray, inter, pdf_DI);

        // uniform sampling indirect lights
        // if (get_random_float() < RussianRoulette)
        // {
        //     Vector3f wi = inter.m->sample(wo, N);
        //     Ray p2o(p, wi);
        //     Intersection inter_indir = intersect(p2o);
        //     float pdf = inter.m->pdf(wi, wo, N);
        //     if (inter_indir.happened && !inter_indir.m->hasEmission())
        //     {
        //         L_indir = castRay_NEE(p2o, depth + 1) * inter.m->eval(wi, wo, N, inter.diffuse_color) * dotProduct(wi, N) / pdf / RussianRoulette;
        //     }
        // }

        // brdf sampling indirect lights (only has 1 pdf)
        // float pdf_brdf = 0.f;
        // if (get_random_float() < RussianRoulette)
        // {
        //     Vector3f wi = inter.m->BRDF_sample(wo, N);
        //     Ray p2o(p, wi);
        //     Intersection inter_indir = intersect(p2o);
        //     if (inter_indir.happened && !inter_indir.m->hasEmission())
        //     {
        //         pdf_brdf = inter.m->BRDF_pdf(wi, wo, N);
        //         L_indir = castRay_NEE(p2o, depth + 1) * inter.m->BRDF_eval(wi, wo, N, inter.diffuse_color) * dotProduct(wi, N) / pdf_brdf / RussianRoulette;
        //     }
        // }

        // separate diffuse and specular pdf
        if (get_random_float() < RussianRoulette)
        {
            Vector3f wi;
            float pdf;
            float u = get_random_float();
            if(u <= inter.m->Kd_avg / (inter.m->Kd_avg + inter.m->Ks_avg) || inter.m -> Ks.x !=0 || inter.m -> Ks.y !=0 || inter.m -> Ks.z !=0){
                wi = inter.m->lambertian_sample(wo, N);
                pdf = inter.m->lambertian_pdf(wi, wo, N);  
            }
            else{
                wi = inter.m->specular_sample(wo, N);
                pdf = inter.m->specular_pdf(wi, wo, N);  
            }
            float cosine = dotProduct(wi, N);
            Vector3f fr = inter.m->BRDF_eval(wi, wo, N, inter.diffuse_color);
            Ray p2o(p, wi);
            Intersection inter_indir = intersect(p2o);
            if (inter_indir.happened && !inter_indir.m->hasEmission() && pdf > EPSILON)
            {   
                L_indir = castRay_NEE(p2o, depth + 1) * fr * cosine / pdf / RussianRoulette;
            }
        }
        // Next Event Estination
        return L_dir + L_indir;

}


/* 
Basic Month Carlo Path Tracing with
    1. uniform sampling
    2. phong model sampling (lambertian&specular)
*/
Vector3f Scene::castRay_BRDF(const Ray &ray, int depth) const
{
    if(depth >= maxDepth)   return Vector3f();
    Intersection inter = intersect(ray);
    
    if (!inter.happened)
    {
        return Vector3f();
    }
    else if (inter.m->hasEmission())
    {
        return inter.m->getEmission();
    }

    Vector3f p = inter.coords;
    Vector3f N = inter.normal;
    Vector3f wo = (ray.origin - p).normalized();
    // backward light remove
    if(dotProduct(wo, N) < 0)    return Vector3f();
    
    Vector3f Lo = Vector3f();

    // deal with reflect&refract obj
    if(inter.m->ior!=1)
    {   
        Vector3f reflectionDirection = normalize(reflect(ray.direction, N));
        Vector3f refractionDirection = normalize(refract(ray.direction, N, inter.m->ior));
        Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ? p - N * EPSILON : p + N * EPSILON;
        Vector3f refractionRayOrig = (dotProduct(refractionDirection, N) < 0) ? p - N * EPSILON : p + N * EPSILON;
        Vector3f reflectionColor = castRay_BRDF(Ray(reflectionRayOrig, reflectionDirection), depth + 1);
        Vector3f refractionColor = castRay_BRDF(Ray(refractionRayOrig, refractionDirection), depth + 1);
        float kr;
        fresnel(ray.direction, N, inter.m->ior, kr);
        Vector3f refle_and_refra =  reflectionColor * kr + refractionColor * (1 - kr) * inter.m->Tr;
        return refle_and_refra;
    }

    // with uniform sampling
    // if(get_random_float() < RussianRoulette)
    // {
    //     Vector3f wi = inter.m->sample(wo, N);
    //     Ray p2o(p, wi);
    //     Intersection inter_next = intersect(p2o);
    //     // if no interseciton
    //     if(!inter_next.happened){
    //         Lo = Vector3f();
    //     }
    //     else {
    //         Vector3f fr = inter.m->eval(wi, wo, N, inter.diffuse_color);
    //         float cosine =  dotProduct(wi, N);
    //         float pdf = inter.m->pdf(wi, wo, N);
    //         // if light
    //         if(inter_next.m->hasEmission()){
    //             Lo = inter_next.m->getEmission() * fr * cosine / pdf; 
    //         }
    //         // if object
    //         else{
    //         Lo = castRay_BRDF(p2o, depth+1) * fr * cosine / pdf;             
    //         }
    //     }
    //     Lo = Lo / RussianRoulette;
    // }


    // with brdf sampling
    if(get_random_float() < RussianRoulette)
    {   
        float u = get_random_float();
        Vector3f wi;
        float pdf;   
    
        if(u <= inter.m->Kd_avg / (inter.m->Kd_avg + inter.m->Ks_avg) || inter.m -> Ks.x !=0 || inter.m -> Ks.y !=0 || inter.m -> Ks.z !=0){
            wi = inter.m->lambertian_sample(wo, N);
            pdf = inter.m->lambertian_pdf(wi, wo, N);  
        }
        else{
            wi = inter.m->specular_sample(wo, N);
            pdf = inter.m->specular_pdf(wi, wo, N);  
        }

        float cosine =  dotProduct(wi, N);
        Vector3f fr = inter.m->BRDF_eval(wi, wo, N, inter.diffuse_color);
        Ray p2o(p, wi);
        Intersection inter_next = intersect(p2o);
        // if no interseciton
        if(!inter_next.happened)
            Lo = Vector3f();
        else if(inter_next.m->hasEmission())
            Lo = inter_next.m->getEmission() * fr * cosine / pdf ;
        else
            Lo = castRay_BRDF(p2o, depth+1) * fr * cosine / pdf;

        Lo = Lo / RussianRoulette;
        
        if(pdf < 1e-3) Lo = Vector3f();
    }
    
    return Lo;
}

// Direct Light + Phong Model Importance Sampling
Vector3f Scene::castRay_MIS(const Ray &ray, int depth) const 
{
    if(depth >= maxDepth)   return Vector3f();
    Intersection inter = intersect(ray);
    
    if (!inter.happened)
    {
        return Vector3f();
    }
    else if (inter.m->hasEmission())
    {
        return inter.m->getEmission();
    }

    Vector3f p = inter.coords;
    Vector3f N = inter.normal;
    Vector3f wo = (ray.origin - p).normalized();
    // backward light remove
    if(dotProduct(wo, N) < 0)    return Vector3f();
    
    float pdf_BRDF = 0, pdf_DI = 0;
    Vector3f Lo, DI;
    
    if(inter.m->ior!=1)
    {   
        Vector3f reflectionDirection = normalize(reflect(ray.direction, N));
        Vector3f refractionDirection = normalize(refract(ray.direction, N, inter.m->ior));
        Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ? p - N * EPSILON : p + N * EPSILON;
        Vector3f refractionRayOrig = (dotProduct(refractionDirection, N) < 0) ? p - N * EPSILON : p + N * EPSILON;
        Vector3f reflectionColor = castRay_MIS(Ray(reflectionRayOrig, reflectionDirection), depth + 1);
        Vector3f refractionColor = castRay_MIS(Ray(refractionRayOrig, refractionDirection), depth + 1);
        float kr;
        fresnel(ray.direction, N, inter.m->ior, kr);
        Vector3f refle_and_refra =  reflectionColor * kr + refractionColor * (1 - kr) * inter.m->Tr;
        return refle_and_refra;
    }

    // DI part
    DI = Scene::Direct_Illumination(ray, inter, pdf_DI);

    // Phong part
    if(get_random_float() < RussianRoulette)
    {   
        float u = get_random_float();
        Vector3f wi;
        float pdf;   
    
        if(u <= inter.m->Kd_avg / (inter.m->Kd_avg + inter.m->Ks_avg) || inter.m -> Ks.x !=0 || inter.m -> Ks.y !=0 || inter.m -> Ks.z !=0){
            wi = inter.m->lambertian_sample(wo, N);
            pdf = inter.m->lambertian_pdf(wi, wo, N);  
        }
        else{
            wi = inter.m->specular_sample(wo, N);
            pdf = inter.m->specular_pdf(wi, wo, N);  
        }
        
        float cosine =  dotProduct(wi, N);
        Vector3f fr = inter.m->BRDF_eval(wi, wo, N, inter.diffuse_color);
        Ray p2o(p, wi);
        Intersection inter_next = intersect(p2o);
        // if no interseciton
        if(!inter_next.happened)
            Lo = Vector3f();
        else if(inter_next.m->hasEmission())
            Lo = inter_next.m->getEmission() * fr * cosine / pdf ;
        else
            Lo = castRay_MIS(p2o, depth+1) * fr * cosine / pdf;

        Lo = Lo / RussianRoulette;
        pdf_BRDF = pdf;
        
        if(pdf < EPSILON) Lo = Vector3f();
    }

        // Multi Importance Sampling
        // Balance heuristic:
        // float pdf_sum = pdf_DI + pdf_BRDF;
        // float weight_DI = pdf_DI / pdf_sum;
        // float weight_BRDF = pdf_BRDF / pdf_sum;
        // Power heuristic:
        // if(pdf_DI < 1e-3 && pdf_BRDF < 1e-3) return Vector3f(0.f);
        if(pdf_DI < EPSILON) return Lo;
        if(pdf_BRDF < EPSILON) return DI;
        float pdf_sum = pow(pdf_DI, 2.0f) + pow(pdf_BRDF, 2.0f);
        float weight_DI = pow(pdf_DI, 2.0f) / pdf_sum;
        float weight_BRDF = pow(pdf_BRDF, 2.0f) / pdf_sum;
        return weight_DI * DI + weight_BRDF * Lo;
    
}
Vector3f Scene::castRay(const Ray &ray, int depth) const 
{
    
}

