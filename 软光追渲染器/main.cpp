#include<iostream>
#include<cmath>
#include<optional>
#include<fstream>
#include<vector>
#include<memory>
#include<algorithm>
struct Vector3 {
	float x;
	float y;
	float z;
	Vector3():x(0),y(0),z(0){}
	Vector3(float x,float y,float z):x(x),y(y),z(z){}
};
Vector3 operator+(Vector3 a, Vector3 b) {
	Vector3 r;
	r.x = a.x + b.x;
	r.y = a.y + b.y;
	r.z = a.z + b.z;
	return r;
}
Vector3 operator-(Vector3 a, Vector3 b) {
	Vector3 r;
	r.x = a.x - b.x;
	r.y = a.y - b.y;
	r.z = a.z - b.z;
	return r;
}
Vector3 operator*(Vector3 v, float s) {
	Vector3 r;
	r.x = v.x * s;
	r.y = v.y * s;
	r.z = v.z * s;
	return r;
}
float dot(Vector3 a, Vector3 b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}
float length(Vector3 a) {
	return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}
Vector3 normalize(Vector3 v) {
	float len = length(v);
	if (len == 0.0f) {
		return Vector3(0, 0, 0);
	}
	Vector3 r;
	r.x = v.x / len;
	r.y = v.y / len;
	r.z = v.z / len;
	return r;
}
Vector3 cross(Vector3 a, Vector3 b) {
	Vector3 r;
	r.x = a.y * b.z - a.z * b.y;
	r.y = a.z * b.x - a.x * b.z;
	r.z = a.x * b.y - a.y * b.x;
	return r;
}
Vector3 operator/(Vector3 v, float a) {
	Vector3 r;
	r.x = v.x / a;
	r.y = v.y / a;
	r.z = v.z / a;
	return r;
}
struct Ray {
	Vector3 origin;
	Vector3 direction;
	Ray(Vector3 o,Vector3 d):origin(o),direction(d){}
};
struct Hit {
	float distance;
	Vector3 point;
	Vector3 normal;
	Vector3 color;
	Hit(float d,Vector3 p,Vector3 n,Vector3 c):distance(d),point(p),normal(n),color(c){}
};
struct Camera {
	float width;
	float height;
	Vector3 origin_light;
	Camera(float w, float h, Vector3 ol) :width(w), height(h), origin_light(ol) {}
	Ray generateRay(float x_, float y_) {
		float aspect = width / height;
		float x = ((x_ / width) * 2.0f - 1.0f) * aspect;
		float y = (1.0f - (y_ / height) * 2.0f);
		return Ray(origin_light, Vector3(x, y, -1.0f));
	}
};
struct AABB {
	Vector3 min_;
	Vector3 max_;
	AABB(){}
	AABB(Vector3 a, Vector3 b) :min_(a), max_(b){}
	float hit(const Ray& ray,float& tmax)const {//判断光线是否打中AABB bounds
		float tx1 = (min_.x - ray.origin.x)/ray.direction.x;
		float tx2 = (max_.x - ray.origin.x) / ray.direction.x;
		float t_minx = std::fmin(tx1, tx2);
		float t_maxx = std::fmax(tx1, tx2);
		float ty1 = (min_.y - ray.origin.y) / ray.direction.y;
		float ty2 = (max_.y - ray.origin.y) / ray.direction.y;
		float t_miny = std::fmin(ty1, ty2);
		float t_maxy = std::fmax(ty1, ty2);
		float tz1 = (min_.z - ray.origin.z) / ray.direction.z;
		float tz2 = (max_.z - ray.origin.z) / ray.direction.z;
		float t_minz = std::fmin(tz1, tz2);
		float t_maxz = std::fmax(tz1, tz2);
		float Tenter = std::fmax(t_minx,std::fmax( t_miny, t_minz));
		float Texit = std::fmin(t_maxx,std::fmin( t_maxy, t_maxz ));
		if (Tenter < Texit && Tenter<tmax&&Texit>0) {//打中盒子
			return std::fmax(Tenter,0.0f);
		}
		else {
			return -1.0f;
		}
	}
};
class Objects {
public:
	Vector3 color;
	Objects(Vector3 c):color(c){}
	virtual std::optional<Hit>intersect(const Ray& ray) const = 0;
	virtual AABB getbounds()const = 0;
};
class Sphere :public Objects {
private:
	Vector3 center;
	float radius;
public:
	Sphere(Vector3 c, float r, Vector3 col) :Objects{ col },center(c), radius(r) {}
	AABB getbounds()const override {//球体的包围盒大小
		Vector3 r(radius, radius, radius);
		return AABB(center - r, center + r);
	}
	std::optional<Hit>intersect(const Ray& ray)const override {
		Vector3 L = ray.origin - center;
		float a = dot(ray.direction, ray.direction);
		float b = 2.0f * dot(L, ray.direction);
		float c = dot(L, L) - radius * radius;
		float v = b * b - 4 * a * c;
		if (v < 0) {
			return std::nullopt;
		}
		float t = -1.0f;
		float sqrt_v = std::sqrt(v);
		float t1 = (-b + sqrt_v) / (2.0f * a);
		float t2 = (-b - sqrt_v) / (2.0f * a);
		if (t1 > 0 && t2 > 0) {
			t = t2;
		}
		else if (t1 > 0) {
			t = t1;
		}
		else if (t2 > 0) {
			t = t2;
		}
		else {
			return std::nullopt;
		}
		Vector3 point = ray.origin + ray.direction*t;
		Vector3 normal = normalize(point - center);
		return Hit(t, point, normal, Vector3(1, 1, 1));
	}
};
class rectangle :public Objects {
private:
	Vector3 min_;
	Vector3 max_;
public:
	rectangle(Vector3 min_point, Vector3 max_point, Vector3 col) :min_(min_point), max_(max_point), Objects{ col } {}
	AABB getbounds()const override {
		return AABB(min_, max_);
	}
	std::optional<Hit>intersect(const Ray& ray)const override {
		float t = -1.0f;
		Vector3 normal;
		if (min_.x == max_.x) {//前后面
			if (std::abs(ray.direction.x)<0.0001f)return std::nullopt;
			t = (min_.x - ray.origin.x) / ray.direction.x;
			normal = Vector3(1, 0, 0);
		}
		else if (min_.y == max_.y) {//左右面
			if (std::abs(ray.direction.y ) < 0.00001f)return std::nullopt;
			t = (min_.y - ray.origin.y) / ray.direction.y;
			normal = Vector3(0, 1, 0);
		}
		else if (min_.z == max_.z) {//上下面
			if (std::abs(ray.direction.z) < 0.00001f)return std::nullopt;
			t = (min_.z - ray.origin.z) / ray.direction.z;
			normal = Vector3(0, 0, 1);
		}
		if (t < 0) {
			return std::nullopt;
		}
		Vector3 point = ray.origin + ray.direction * t;
		if (point.x<min_.x - 1e-3f || point.x>max_.x + 1e-3f ||
			point.y<min_.y - 1e-3f || point.y>max_.y + 1e-3f ||
			point.z<min_.z - 1e-3f || point.z>max_.z + 1e-3f) {
			return std::nullopt;
		}
		return Hit(t, point, normal, color);
	}
};
class Box:public Objects {
private:
	Vector3 min_p;
	Vector3 max_p;
	rectangle side[6];
public:
	Box(Vector3 ip, Vector3 xp, Vector3 col) : min_p(ip),max_p(xp),Objects{col}, side{
		rectangle(min_p,Vector3(max_p.x,min_p.y,max_p.z),col),//下
		rectangle(Vector3(min_p.x,max_p.y,min_p.z),max_p,col),//上
		rectangle(min_p,Vector3(min_p.x,max_p.y,max_p.z),col),//左
		rectangle(Vector3(max_p.x,min_p.y,min_p.z),max_p,col),//右
		rectangle(min_p,Vector3(max_p.x,max_p.y,min_p.z),col),//后
		rectangle(Vector3(min_p.x,min_p.y,max_p.z),max_p,col)//前
	    }{}
	AABB getbounds()const override {
		return AABB(min_p, max_p);
	}
	std::optional<Hit>intersect(const Ray& ray)const override {
		std::optional<Hit>closet_hit = std::nullopt;
		float min_dist = 1e9;
		for (const auto& face : side) {
			auto h = face.intersect(ray);
			if (h&&h->distance < min_dist) {
				min_dist = h->distance;
				closet_hit = h;
			}
		}
		return closet_hit;
	}
};
class BVH :public Objects {
private:
	AABB box;
	std::shared_ptr<BVH>left;
	std::shared_ptr<BVH>right;
	std::vector<std::shared_ptr<Objects>>obj;
	std::optional<Hit>traverse(const Ray& ray, float& tmax)const {
		float t = box.hit(ray, tmax);
		if (t < 0) {
			return std::nullopt;
		}
		std::optional<Hit>best = std::nullopt;
		if (left == NULL || right == NULL) {
			for (const auto& ob : obj) {
				auto h = ob->intersect(ray);
				if (h && h->distance < tmax) {
					tmax = h->distance;
					best = h;
				}
			}
		}
		else {
			auto a = left->traverse(ray,tmax);
			auto b = right->traverse(ray,tmax);
			if (b)return b;
			return a;
		}
		return best;
	}
public:
	BVH(std::vector<std::shared_ptr<Objects>>bj) :Objects(Vector3(1, 1, 1)), obj(bj) {
		box = obj[0]->getbounds();
		for (int i = 1; i < obj.size(); i++) {
			box = merge(box, obj[i]->getbounds());
		}
		if (obj.size() <= 5)return;
		int n = obj.size();
		std::vector<std::shared_ptr<Objects>>L(obj.begin(), obj.begin() + n / 2);//左孩子
		std::vector<std::shared_ptr<Objects>>R(obj.begin() + n / 2, obj.end());//右孩子
		left = std::make_shared<BVH>(L);
		right = std::make_shared<BVH>(R);
	}
	std::optional<Hit>intersect(const Ray& ray)const override {
		float tmax = 1e30;
		return traverse(ray, tmax);
	}
	AABB getbounds()const override {
		return box;
	}
	AABB merge(const AABB& a,const AABB& b) {
		Vector3 in(std::fmin(a.min_.x, b.min_.x),
			std::fmin(a.min_.y, b.min_.y),
			std::fmin(a.min_.z, b.min_.z));
		Vector3 ax(std::fmax(a.max_.x, b.max_.x),
			std::fmax(a.max_.y, b.max_.y),
			std::fmax(a.max_.z, b.max_.z));
		return AABB(in, ax);
	}
	

};
Vector3 traceray(Ray& ray, std::vector<std::shared_ptr<Objects>>scene, int depth, Vector3 lightposition) {
	std::optional<Hit>closet_hit = std::nullopt;
	float min_dist = 1e9;
	for (const auto& obj : scene) {
		auto h = obj->intersect(ray);
		if (h) {
			if (h->distance < min_dist) {
				min_dist = h->distance;
				closet_hit = h;
			}
		}
	}
	if (!closet_hit)return Vector3(0, 0, 0);
	Vector3 finalpoint = closet_hit->point;
	Vector3 finalnormal = closet_hit->normal;
	Vector3 direct = normalize(lightposition - finalpoint);
	float color = dot(finalnormal, direct);
	if (color < 0) {
		color = 0.0f;
	}
	Vector3 dir = normalize(lightposition - finalpoint);
	Ray shadowRay(finalpoint + finalnormal * 0.01f, dir);
	for (const auto& obje : scene) {
		auto h1 = obje->intersect(shadowRay);
		float distlight = length(lightposition - finalpoint);
		if (h1&&h1->distance<distlight) {
			return Vector3(0, 0, 0);
		}
	}
	if (depth > 5) {
		return Vector3(0, 0, 0);
	}
	return closet_hit->color*color;
	/*Vector3 reflectdir = normalize(ray.direction - finalnormal * (2 * dot(ray.direction, finalnormal)));
	Ray reflectray(finalpoint + finalnormal * 0.0001f, reflectdir);
	Vector3 reflectcolor = traceray(reflectray, scene, depth + 1, lightposition);
	return Vector3(color, color, color) * 0.8f + reflectcolor * 0.2f;*/
}
int main() {
	std::ofstream out("image.ppm");
	out << "P3\n";
	out << "1920 1080\n";
	out << "255\n";
	std::vector<std::shared_ptr<Objects>>scene;
	scene.push_back(std::make_shared<Sphere>(Vector3(0,-2,-15),3.0f,Vector3(1,1,1)));//最右边
	scene.push_back(std::make_shared<Box>(Vector3(-17, -5, -27), Vector3(-13, 0, -23), Vector3(1,1,1)));
	scene.push_back(std::make_shared<Sphere>(Vector3(-15, 2, -25), 2.0f, Vector3(1, 0, 0)));//左后
	scene.push_back(std::make_shared<Box>(Vector3(-25, -5, -23), Vector3(-21, 3, -19), Vector3(1,1,1)));
	//scene.push_back(std::make_shared<Sphere>(Vector3(-15, -2.5, -15), 2.5f, Vector3(1, 1, 1)));//左前
	scene.push_back(std::make_shared<rectangle>(Vector3(-30, -5, -35), Vector3(30, 20, -35), Vector3(1,1,1))); // 后墙
	scene.push_back(std::make_shared<rectangle>(Vector3(-30, -5, -35), Vector3(-30, 20, 10), Vector3(1,1,1))); // 左墙
	scene.push_back(std::make_shared<rectangle>(Vector3(-30, -5, -35), Vector3(30, -5, 10), Vector3(1,1,1))); // 地板
	auto root = std::make_shared<BVH>(scene);
	scene.clear();
	scene.push_back(root);
	Camera c(1920.0f, 1080.f, Vector3(0, 0, 0));
	Vector3 lightposition(5, 5, 0);
	for (int i = 0; i < (int)c.height; i++) {
		for (int j = 0; j < (int)c.width; j++) {
			Vector3 color(0, 0, 0);
			for (int x = 0; x < 2; x++) {
				for (int y = 0; y < 2; y++) {
					float px = j + 0.25f + x * 0.5f;
					float py = i + 0.25f + y * 0.5f;
					Ray ray = c.generateRay(px, py);
					color = color + traceray(ray, scene, 0, lightposition);
				}
			}
			color = color / 4;
			int r = (int)(color.x * 255.0f);
			int g = (int)(color.y * 255.0f);
			int b = (int)(color.z * 255.0f);
			out << r << " " << g << " " << b << "\n";
		}
	}
	out.close();
	return 0;
}