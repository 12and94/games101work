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
    
    Vector3f L_indir={ 0.0,0.0,0.0 };
    Vector3f L_dir={ 0.0,0.0,0.0 };
    Intersection intersection = Scene::intersect(ray);
    if (!intersection.happened) {
        return Vector3f(0.0,0.0,0.0);
    }
    if(intersection.m->hasEmission()){
        if (depth == 0) {
            return intersection.m->getEmission();
        }
        else return Vector3f(0.0,0.0,0.0);
    }
    Vector3f hitColor = this->backgroundColor;
    Intersection light_inter;
    float light_pdf;
    sampleLight(light_inter,light_pdf);
    Vector3f obj_p=intersection.coords;
    Vector3f obj_n=intersection.normal.normalized();
    Vector3f wo=ray.direction;

    Vector3f light_p=light_inter.coords;
    Vector3f light_n=light_inter.normal.normalized();
    
    Vector3f ws=(obj_p-light_p).normalized();
    float dis=(obj_p-light_p).norm();;
    float dis2 = dotProduct((obj_p-light_p),(obj_p-light_p));
    Ray light_obj(light_p,ws);
    Intersection ltos=Scene::intersect(light_obj);

    if(ltos.distance>dis-EPSILON){
        L_dir=light_inter.emit*intersection.m->eval(wo,-ws,obj_n)*dotProduct(-ws,obj_n)*dotProduct(ws,light_n)/dis2/light_pdf;
    }
    float test=get_random_float();
    if(test<RussianRoulette){
        Vector3f wi=intersection.m->sample(wo,obj_n).normalized();
        Ray obj_r(obj_p,wi);
        Intersection otos=Scene::intersect(obj_r);
        if(otos.happened&&!otos.m->hasEmission()){
            if(intersection.m->pdf(wo,wi,obj_n)>EPSILON){
                L_indir=castRay(obj_r,depth+1)*intersection.m->eval(wo,wi,obj_n)*dotProduct(wi,obj_n)/(float)intersection.m->pdf(wo,wi,obj_n)/RussianRoulette;
            }
            
        }
    }

    return L_dir+L_indir;
    
    // TO DO Implement Path Tracing Algorithm here

}