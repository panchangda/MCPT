//
// Created by LEI XU on 5/16/19.
//
#pragma once

#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "Vector.hpp"
#include "global.hpp"



enum MaterialType { DIFFUSE, PHONG };

class Material{
private:

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

    Vector3f toWorld(const Vector3f &a, const Vector3f &N){
        Vector3f B, C;
        if (std::fabs(N.x) > std::fabs(N.y)){
            float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
            C = Vector3f(N.z * invLen, 0.0f, -N.x *invLen);
        }
        else {
            float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
            C = Vector3f(0.0f, N.z * invLen, -N.y *invLen);
        }
        B = crossProduct(C, N);
        return a.x * B + a.y * C + a.z * N;
    }

public:
    MaterialType m_type;
    //Vector3f m_color;
    Vector3f m_emission=Vector3f();
    // Ni
    float ior=1;
    Vector3f Kd, Ks, Tr;
    float Ks_coe, Kd_coe;
    float Ks_avg, Kd_avg;
    // Ns
    float specularExponent;

    
    //Texture tex;
    bool useTex = false;
    unsigned char *Tex;
    int nChannels, height, width;

    inline Material(MaterialType t=DIFFUSE, Vector3f e=Vector3f(0,0,0));
    inline MaterialType getType();
    //inline Vector3f getColor();
    inline Vector3f getColorAt(double u, double v);
    inline void loadTex(unsigned char* _Tex, int w, int h, int n){ 
        if(_Tex){ Tex = _Tex; width=w; height=h; nChannels=n; useTex=true; } 
        else{ printf("loading texture failed!\n");}
    };
    inline Vector3f getEmission();
    inline bool hasEmission();

    // sample a ray by Material properties
    inline Vector3f sample(const Vector3f &wi, const Vector3f &N);

    // given a ray, calculate the PdF of this ray
    inline float pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);
    // given a ray, calculate the contribution of this ray
    inline Vector3f eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);
    inline Vector3f eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N, Vector3f texture_color);

    // brdfs
    inline float BRDF_pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);
    inline Vector3f BRDF_sample(const Vector3f &wi, const Vector3f &N);
    inline Vector3f BRDF_eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N, Vector3f texture_color);
    
    inline Vector3f BackwardJudge(const Vector3f &wi, const Vector3f &N);

    inline float lambertian_pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);
    inline float specular_pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);
    inline Vector3f lambertian_sample(const Vector3f &wi, const Vector3f &N);
    inline Vector3f specular_sample(const Vector3f &wi, const Vector3f &N);
};

Material::Material(MaterialType t, Vector3f e){
    m_type = t;
    //m_color = c;
    m_emission = e;
}

MaterialType Material::getType(){return m_type;}
///Vector3f Material::getColor(){return m_color;}
Vector3f Material::getEmission() {return m_emission;}
bool Material::hasEmission() {
    if (m_emission.norm() > EPSILON) return true;
    else return false;
}

Vector3f Material::getColorAt(double u, double v) {
    if(!useTex) return Vector3f();
    if(u > 1){
        u= fmodf(u,1);
    }
    if(v > 1){
        v = fmodf(v,1);
    }
    if(u < 0){
        u = 1.f - fmodf(-u,1);
    }
    if(v < 0){
        v = 1.f - fmodf(-v,1);
    }
    int w = width*u;
    int h = height*(1-v);
    w = std::max(0, std::min(width-1, w));
    h = std::max(0, std::min(height-1, h));
    // printf("u v: %f %f   ->   w h: %d %d\n",u, v, w, h);
    float r = Tex[nChannels * width * h + nChannels*w + 0] / 255.f;
    float g = Tex[nChannels * width * h + nChannels*w + 1] / 255.f;
    float b = Tex[nChannels * width * h + nChannels*w + 2] / 255.f;
    // printf("r g b: %f %f %f \n", r, g, b);
    return  Vector3f(r, g, b);
}
Vector3f Material::BackwardJudge(const Vector3f &wi, const Vector3f &N)
{
    float cosalpha = dotProduct(wi, N);
    if(cosalpha > 0)
        return Vector3f(1.0f);
    else
        return Vector3f(0.f);
}
Vector3f Material::sample(const Vector3f &wi, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {   
            // uniform sample on the hemisphere
            float x_1 = get_random_float(), x_2 = get_random_float();
            float z = std::fabs(1.0f - 2.0f * x_1);
            float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
            Vector3f localRay(r*std::cos(phi), r*std::sin(phi), z);
            return toWorld(localRay, N);
            
            break;
        }
        case PHONG:
        {
            float x_1 = get_random_float(), x_2 = get_random_float();
            float z = std::fabs(1.0f - 2.0f * x_1);
            float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
            Vector3f localRay(r*std::cos(phi), r*std::sin(phi), z);
            return toWorld(localRay, N);
            break;
        }
    }
}

float Material::pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
        // uniform sample probability 1 / (2 * PI)
        return 0.5f / M_PI;

    // switch(m_type){
    //     case DIFFUSE:
    //     {
    //         // uniform sample probability 1 / (2 * PI)
    //         if (dotProduct(wo, N) > 0.0f)
    //             return 0.5f / M_PI;
    //         else
    //             return 0.0f;
    //         break;
    //     }
    //     case PHONG:
    //     {
    //         // combine diffue and specular sampling 
    //         // (1-ks) * cos(theta_i) / PI + ks * (n+1)/2PI * cos(alpha)[exp=n]
    //         Vector3f halfwayDir = (wi+wo).normalized(); 
    //         if (dotProduct(wo, N) > 0.0f)
    //             return  (1-Ks_avg) * dotProduct(wo, N)/ M_PI + 
    //                     Ks_avg * (specularExponent+1) / 2* M_PI * pow( std::max(dotProduct(halfwayDir, N), 0.f), specularExponent);
    //         else
    //             return 0.0f;
    //         break;
    //     }
    // }
}
Vector3f Material::eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:   // BRDF Diffuse
        {
            // calculate the contribution of diffuse model
            Vector3f diffuse = Kd / M_PI;
            return diffuse;
            break;
        }
        case PHONG: // Phong Reflectance Model
        {
            Vector3f diffuse = Kd / M_PI;
            Vector3f halfwayDir = (wi+wo).normalized(); 
            Vector3f specular = Ks * (specularExponent+2) / (2*M_PI) * pow( std::max(dotProduct(halfwayDir, N), 0.f), specularExponent);
            return diffuse + specular;
            break;
        }
    }
}

Vector3f Material::eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N, Vector3f texture_color){
    switch(m_type){
        case DIFFUSE:   // BRDF Diffuse
        {
            // calculate the contribution of diffuse model
            if(texture_color.norm()<EPSILON) texture_color = Kd;
            Vector3f diffuse = texture_color / M_PI;
            return diffuse;
            break;
        }
        case PHONG: // Phong Reflectance Model
        {   
            if(texture_color.norm()<EPSILON) texture_color = Kd;
            Vector3f diffuse = texture_color / M_PI;
            Vector3f halfwayDir = (wi+wo).normalized(); 
            Vector3f specular = Ks * (specularExponent+2) / (2*M_PI) * pow( std::max(dotProduct(halfwayDir, N), 0.f), specularExponent);
            return diffuse + specular;
            break;
        }
    }
}



Vector3f Material::BRDF_eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N, Vector3f texture_color){
    switch(m_type){
        case DIFFUSE:   // BRDF Diffuse
        {
            // calculate the contribution of diffuse model
            if(texture_color.norm()<EPSILON) texture_color = Kd;
            Vector3f diffuse = texture_color / M_PI;
            return diffuse;
            break;
        }
        case PHONG: // Phong Reflectance Model
        {   
            if(texture_color.norm()<EPSILON) texture_color = Kd;
            Vector3f diffuse = texture_color / M_PI;
            Vector3f halfwayDir = (wi+wo).normalized(); 
            Vector3f specular = Ks * (specularExponent+2) / (2*M_PI) * pow( std::max(dotProduct(halfwayDir, N), 0.f), specularExponent);
            return diffuse + specular;
            break;
        }
    }
}

Vector3f Material::BRDF_sample(const Vector3f &wi, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // cosine weighted sampling
            float x_1 = get_random_float(), x_2 = get_random_float();
            float theta = std::acos(std::sqrt(1.f - x_1));
            float phi = 2*M_PI* x_2;
            Vector3f localRay(std::sin(theta) * std::cos(phi), std::sin(theta)*std::sin(phi), std::cos(theta));
            return toWorld(localRay, N);
            break;
        }
        case PHONG:
        {
            float k = get_random_float();
            if(k >= Ks_coe){
                // cosine weighted sampling AKA diffuse brdf sampling
                float x_1 = get_random_float(), x_2 = get_random_float();
                float theta = std::acos(std::sqrt(1.f - x_1));
                float phi = 2*M_PI* x_2;
                Vector3f localRay(std::sin(theta) * std::cos(phi), std::sin(theta)*std::sin(phi), std::cos(theta));
                return toWorld(localRay, N);
            }
            else{
                // specular brdf sampling
                float x_1 = get_random_float(), x_2 = get_random_float();
                float theta = std::acos(pow(x_1, 1.0f/(specularExponent + 1))); 
                float phi = 2 * M_PI * x_2;
                Vector3f localRay(std::sin(theta)*std::cos(phi), std::sin(theta)*std::sin(phi), std::cos(theta));
                return toWorld(localRay, N);
            }
            break;
        }
    }
}
float Material::BRDF_pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // cosine weighted sampling:
            // cos(theta_i) / PI
            return dotProduct(wi, N)/ M_PI;
            break;
        }
        case PHONG:
        {
            // combine diffue and specular sampling 
            // (1-ks) * cos(theta_i) / PI + ks * (n+1)/2PI * cos(alpha)[exp=n]
            Vector3f halfwayDir = (wi+wo).normalized(); 

            return  (1-Ks_coe) * dotProduct(wi, N)/ M_PI + 
                        Ks_coe * (specularExponent+1) / (2* M_PI) * pow( std::max(dotProduct(halfwayDir, N), 0.f), specularExponent);
            break;
        }
    }
}

float Material::lambertian_pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    return dotProduct(wi, N)/ M_PI;
}
Vector3f Material::lambertian_sample(const Vector3f &wi, const Vector3f &N){
    float x_1 = get_random_float(), x_2 = get_random_float();
    float theta = std::acos(std::sqrt(1.0f - x_1));
    float phi = 2*M_PI* x_2;
    Vector3f localRay(std::sin(theta) * std::cos(phi), std::sin(theta)*std::sin(phi), std::cos(theta));
    return toWorld(localRay, N);
}
float Material::specular_pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    Vector3f halfwayDir = normalize(wi + wo); 
    return  (specularExponent+1) / (2* M_PI) * pow( std::max(dotProduct(halfwayDir, N), 0.f), specularExponent);
}
Vector3f Material::specular_sample(const Vector3f &wi, const Vector3f &N){
    float x_1 = get_random_float(), x_2 = get_random_float();
    float theta = std::acos(pow(x_1, 1.0f/(specularExponent + 1.0f))); 
    float phi = 2 * M_PI * x_2;
    Vector3f localRay(std::sin(theta)*std::cos(phi), std::sin(theta)*std::sin(phi), std::cos(theta));
    return toWorld(localRay, N);
}

#endif //RAYTRACING_MATERIAL_H
