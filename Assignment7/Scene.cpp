//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Vector3f L_dir, L_indir;
    
    Intersection intersection = Scene::intersect(ray);
    Material *m = intersection.m;
    Object *hitObject = intersection.obj;
    Vector3f hitColor = this->backgroundColor;
    Vector2f uv;
    uint32_t index = 0;
    float pdf;

    if(intersection.happened) {
        if(hitObject->hasEmit()){
            if (depth == 0){
                return m->getEmission();
            }
            return L_dir;
        } 
        // std::cout << "castRay:3" << std::endl;
        Vector3f hitPoint = intersection.coords;
        Vector3f N = intersection.normal; // normal
        Vector2f st; // st coordinates
        hitObject->getSurfaceProperties(hitPoint, ray.direction, index, uv, N, st);
        Intersection light;

        sampleLight(light, pdf);
        Vector3f p2light_dir = light.coords - hitPoint;
        float p2light_dis = p2light_dir.norm();
        p2light_dir = normalize(p2light_dir);
        Intersection p2light_inter = Scene::intersect(Ray(hitPoint, p2light_dir));

        if (p2light_inter.happened){
            if ((p2light_inter.coords - light.coords).norm() <= EPSILON){
                L_dir = p2light_inter.m->getEmission() * m->eval(-ray.direction, p2light_dir, N) * dotProduct(N, p2light_dir) * dotProduct(light.normal, -p2light_dir) / p2light_dis / p2light_dis / pdf;
            }
        }
        
        if(get_random_float() <= RussianRoulette){
            switch (m->getType()) {
                case DIFFUSE:{
                    Vector3f outdir = normalize(m->sample(-ray.direction, N));
                    pdf = m->pdf(-ray.direction, outdir, N);
                    Ray outray(hitPoint, outdir);
                    L_indir = castRay(outray, depth + 1) * m->eval(-ray.direction, outdir, N) * dotProduct(N, outdir) / pdf / RussianRoulette;
                    break;
                }
                default:{
                    std::cout<< "default type?" << std::endl;
                }
            }
        }
    }

    
    return L_dir + L_indir;
}