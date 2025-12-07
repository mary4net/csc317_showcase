#define SDL_MAIN_HANDLED
#include "Camera.h"
#include "Light.h"
#include "Material.h"
#include "PointLight.h"
#include "Triangle.h"
#include "TriangleSoup.h"
#include "catmull_clark.h"
#include "mesh_builders.h"
#include "mesh_types.h"
#include "raycolor.h"
#include "viewing_ray.h"
#include "write_ppm.h"
#include <SDL2/SDL.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <future>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

namespace {

struct OrbitalCamera {
  double yaw = 0.0;
  double pitch = 0.05;
  double distance = 2.2;
  Eigen::Vector3d target = Eigen::Vector3d(0.0, 1.0, 0.0);
  double vfov = 60.0 * M_PI / 180.0;
};

struct SceneBuild {
  std::vector<std::shared_ptr<Object>> objects;
  std::vector<std::shared_ptr<Light>> lights;
  std::shared_ptr<PointLight> flashlight;
};

Eigen::Vector3d camera_forward(double yaw, double pitch) {
  Eigen::Vector3d f(std::sin(yaw) * std::cos(pitch), std::sin(pitch),
                    std::cos(yaw) * std::cos(pitch));
  return f.normalized();
}

Eigen::Vector3d camera_eye(const OrbitalCamera &o) {
  Eigen::Vector3d f = camera_forward(o.yaw, o.pitch);
  return o.target - f * o.distance;
}

void fill_camera(const OrbitalCamera &o, Camera &cam) {
  Eigen::Vector3d f = camera_forward(o.yaw, o.pitch);
  Eigen::Vector3d e = o.target - f * o.distance;
  Eigen::Vector3d v(0.0, 1.0, 0.0);
  Eigen::Vector3d w = -f;
  Eigen::Vector3d u = v.cross(w).normalized();
  v = w.cross(u).normalized(); // keep v pointing up relative to w/u
  cam.e = e;
  cam.w = w;
  cam.u = u;
  cam.v = v;
  cam.d = 1.0;
  cam.height = 2.0 * cam.d * std::tan(o.vfov * 0.5);
  cam.width = cam.height * (16.0 / 9.0);
}

std::shared_ptr<Material> make_material(const Eigen::Vector3d &ka,
                                        const Eigen::Vector3d &kd,
                                        const Eigen::Vector3d &ks,
                                        const Eigen::Vector3d &km,
                                        double phong) {
  auto m = std::make_shared<Material>();
  m->ka = ka;
  m->kd = kd;
  m->ks = ks;
  m->km = km;
  m->phong_exponent = phong;
  return m;
}

std::shared_ptr<TriangleSoup> quad_mesh_to_soup(
    const Mesh &mesh, const std::shared_ptr<Material> &mat) {
  auto soup = std::make_shared<TriangleSoup>();
  for (int f = 0; f < mesh.F.rows(); ++f) {
    const int a = mesh.F(f, 0);
    const int b = mesh.F(f, 1);
    const int c = mesh.F(f, 2);
    const int d = mesh.F(f, 3);
    Eigen::Vector3d va = mesh.V.row(a);
    Eigen::Vector3d vb = mesh.V.row(b);
    Eigen::Vector3d vc = mesh.V.row(c);
    Eigen::Vector3d vd = mesh.V.row(d);
    auto t0 = std::make_shared<Triangle>();
    t0->corners = std::make_tuple(va, vb, vc);
    t0->material = mat;
    auto t1 = std::make_shared<Triangle>();
    t1->corners = std::make_tuple(va, vc, vd);
    t1->material = mat;
    soup->triangles.push_back(t0);
    soup->triangles.push_back(t1);
  }
  soup->material = mat;
  return soup;
}

Mesh apply_transform(const Mesh &mesh, const Eigen::Vector3d &t) {
  Mesh out = mesh;
  out.V = mesh.V.rowwise() + t.transpose();
  return out;
}

Mesh subdivide_mesh(const Mesh &mesh, int iterations) {
  Mesh out;
  catmull_clark(mesh.V, mesh.F, iterations, out.V, out.F);
  return out;
}

SceneBuild build_scene() {
  SceneBuild S;

  // Materials
  // Pale wall tint (near white with a hint of color)
  Eigen::Vector3d wall_color(0.82, 0.84, 0.88); // subtle blue/pink mixed to near-white
  auto wall_mat = make_material(Eigen::Vector3d(0.02, 0.02, 0.02),
                                wall_color,
                                Eigen::Vector3d(0.04, 0.04, 0.04),
                                Eigen::Vector3d(0.0, 0.0, 0.0), 8.0);
  auto table_mat = make_material(Eigen::Vector3d(0.05, 0.04, 0.03),
                                 Eigen::Vector3d(0.6, 0.45, 0.3),
                                 Eigen::Vector3d(0.05, 0.05, 0.05),
                                 Eigen::Vector3d(0.0, 0.0, 0.0), 20.0);
  auto metal_mat = make_material(Eigen::Vector3d(0.05, 0.05, 0.05),
                                 Eigen::Vector3d(0.1, 0.1, 0.1),
                                 Eigen::Vector3d(0.9, 0.9, 0.9),
                                 Eigen::Vector3d(0.8, 0.8, 0.8), 120.0);
  auto mirror_mat = make_material(Eigen::Vector3d(0.0, 0.0, 0.0),
                                  Eigen::Vector3d(0.01, 0.01, 0.01),
                                  Eigen::Vector3d(0.99, 0.99, 0.99),
                                  Eigen::Vector3d(1.0, 1.0, 1.0), 300.0);

  // Room and table
  Mesh room = build_room_mesh();
  Mesh table = apply_transform(build_table_mesh(), Eigen::Vector3d(1.6, 0.0, -1.0));
  S.objects.push_back(quad_mesh_to_soup(room, wall_mat));
  S.objects.push_back(quad_mesh_to_soup(table, table_mat));

  // Cube on table (subdivided once)
  Mesh cube = subdivide_mesh(build_cube_mesh(0.6), 1);
  cube = apply_transform(cube, Eigen::Vector3d(1.6, 1.45, -1.0));
  S.objects.push_back(quad_mesh_to_soup(cube, metal_mat));

  // Mirror on back wall
  Mesh mirror = apply_transform(build_mirror_mesh(1.6, 1.0),
                                Eigen::Vector3d(0.0, 1.6, -2.99));
  S.objects.push_back(quad_mesh_to_soup(mirror, mirror_mat));

  // Lights
  auto overhead = std::make_shared<PointLight>();
  overhead->p = Eigen::Vector3d(0.0, 2.6, 0.0);
  overhead->I = Eigen::Vector3d(0.2, 0.2, 0.2); // dim fill so movable light dominates shadows
  S.lights.push_back(overhead);

  S.flashlight = std::make_shared<PointLight>();
  S.flashlight->p = Eigen::Vector3d(0.0, 1.3, 1.0);
  S.flashlight->I = Eigen::Vector3d(0.9, 0.8, 0.7); // still soft but brighter than fill
  S.lights.push_back(S.flashlight);

  return S;
}

struct RenderResult {
  std::vector<unsigned char> pixels;
  int width = 0;
  int height = 0;
};

RenderResult render_frame(const SceneBuild &scene,
                          const Camera &cam,
                          int width,
                          int height) {
  RenderResult result;
  result.width = width;
  result.height = height;
  result.pixels.resize(3 * width * height);
  auto to_uc = [](double s) {
    return static_cast<unsigned char>(
        255.0 * std::max(std::min(s, 1.0), 0.0));
  };

  #pragma omp parallel for
  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      Eigen::Vector3d rgb(0, 0, 0);
      Ray ray;
      viewing_ray(cam, i, j, width, height, ray);
      raycolor(ray, 1.0, scene.objects, scene.lights, 0, rgb);
      const int idx = 3 * (j + width * i);
      result.pixels[idx + 0] = to_uc(rgb(0));
      result.pixels[idx + 1] = to_uc(rgb(1));
      result.pixels[idx + 2] = to_uc(rgb(2));
    }
  }
  return result;
}

void clamp_inside(OrbitalCamera &c) {
  c.target.x() = std::clamp(c.target.x(), -2.7, 2.7);
  c.target.z() = std::clamp(c.target.z(), -2.7, 2.7);
  c.target.y() = std::clamp(c.target.y(), 0.3, 2.7);
  c.distance = std::clamp(c.distance, 0.4, 6.0);
}

} // namespace

int main(int /*argc*/, char * /*argv*/[]) {
  SDL_SetMainReady();
  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    std::cerr << "Failed to initialize SDL2: " << SDL_GetError() << "\n";
    return 1;
  }

  const int base_width = 640;
  const int base_height = 360;
  SDL_Window *window =
      SDL_CreateWindow("Realtime ray tracing (CPU)", SDL_WINDOWPOS_CENTERED,
                       SDL_WINDOWPOS_CENTERED, base_width, base_height,
                       SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
  if (!window) {
    std::cerr << "Failed to create window: " << SDL_GetError() << "\n";
    SDL_Quit();
    return 1;
  }
  SDL_Renderer *renderer =
      SDL_CreateRenderer(window, -1,
                         SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  if (!renderer) {
    std::cerr << "Failed to create renderer: " << SDL_GetError() << "\n";
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }

  SceneBuild scene = build_scene();
  OrbitalCamera orbit;
  orbit.yaw = 0.0;
  orbit.pitch = 0.05;
  orbit.distance = 2.2;
  orbit.target = Eigen::Vector3d(0.0, 1.0, 0.0);
  clamp_inside(orbit);

  int width = base_width;
  int height = base_height;
  SDL_Texture *texture = SDL_CreateTexture(
      renderer, SDL_PIXELFORMAT_RGB24, SDL_TEXTUREACCESS_STREAMING, width,
      height);

  auto update_flashlight = [&](const Camera &cam) {
    Eigen::Vector3d up = cam.v.normalized();
    scene.flashlight->p = cam.e - 0.2 * up;
  };

  auto request_render = [&](OrbitalCamera &orb, std::future<RenderResult> &job,
                            bool &inflight) {
    clamp_inside(orb);
    Camera cam;
    fill_camera(orb, cam);
    update_flashlight(cam);
    inflight = true;
    const int w = width;
    const int h = height;
    job = std::async(std::launch::async, render_frame, std::cref(scene), cam,
                     w, h);
  };

  bool running = true;
  bool rotating = false;
  int last_key_rotate = 0;
  bool inflight = false;
  std::future<RenderResult> render_job;
  RenderResult latest;
  request_render(orbit, render_job, inflight);

  while (running) {
    SDL_Event ev;
    bool camera_changed = false;
    while (SDL_PollEvent(&ev)) {
      switch (ev.type) {
      case SDL_QUIT:
        running = false;
        break;
      case SDL_WINDOWEVENT:
        if (ev.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
          width = ev.window.data1;
          height = ev.window.data2;
          if (texture) {
            SDL_DestroyTexture(texture);
          }
          texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGB24,
                                      SDL_TEXTUREACCESS_STREAMING, width,
                                      height);
          latest = {};
          camera_changed = true;
        }
        break;
      case SDL_MOUSEBUTTONDOWN:
        // Mouse drag disabled; no-op
        break;
      case SDL_MOUSEBUTTONUP:
        // Mouse drag disabled; no-op
        break;
      case SDL_MOUSEMOTION:
      case SDL_MOUSEWHEEL:
        // Mouse-driven orbit/zoom disabled
        break;
      case SDL_KEYDOWN:
        if (ev.key.keysym.sym == SDLK_ESCAPE) {
          running = false;
        } else if (ev.key.keysym.sym == SDLK_LEFT ||
                   ev.key.keysym.sym == SDLK_RIGHT ||
                   ev.key.keysym.sym == SDLK_UP ||
                   ev.key.keysym.sym == SDLK_DOWN) {
          if (!rotating) {
            const Eigen::Vector3d eye = camera_eye(orbit);
            rotating = true;
            last_key_rotate = ev.key.keysym.sym;
            const double step_ang = 0.08;
            if (ev.key.keysym.sym == SDLK_LEFT) {
              orbit.yaw += step_ang;
            } else if (ev.key.keysym.sym == SDLK_RIGHT) {
              orbit.yaw -= step_ang;
            } else if (ev.key.keysym.sym == SDLK_UP) {
              orbit.pitch += step_ang;
            } else if (ev.key.keysym.sym == SDLK_DOWN) {
              orbit.pitch -= step_ang;
            }
            orbit.pitch = std::clamp(orbit.pitch, -1.3, 1.3);
            const Eigen::Vector3d new_f = camera_forward(orbit.yaw, orbit.pitch);
            orbit.target = eye + new_f * orbit.distance;
            camera_changed = true;
          }
        }
        break;
      case SDL_KEYUP:
        if (ev.key.keysym.sym == last_key_rotate) {
          rotating = false;
          last_key_rotate = 0;
        }
        break;
      default:
        break;
      }
    }

    const Uint8 *keys = SDL_GetKeyboardState(nullptr);
    Eigen::Vector3d fwd = camera_forward(orbit.yaw, orbit.pitch);
    Eigen::Vector3d up(0.0, 1.0, 0.0);
    Eigen::Vector3d right = fwd.cross(up).normalized();
    up = right.cross(fwd).normalized();
    Eigen::Vector3d flat_fwd = fwd;
    flat_fwd.y() = 0.0;
    if (flat_fwd.squaredNorm() < 1e-6) {
      flat_fwd = Eigen::Vector3d(0.0, 0.0, -1.0);
    }
    flat_fwd.normalize();
    const double move_speed = 0.02 * orbit.distance;
    Eigen::Vector3d move = Eigen::Vector3d::Zero();
    if (keys[SDL_SCANCODE_W])
      move += flat_fwd;
    if (keys[SDL_SCANCODE_S])
      move -= flat_fwd;
    if (keys[SDL_SCANCODE_A])
      move -= right;
    if (keys[SDL_SCANCODE_D])
      move += right;
    if (keys[SDL_SCANCODE_SPACE])
      move += Eigen::Vector3d(0.0, 1.0, 0.0);
    if (keys[SDL_SCANCODE_LCTRL])
      move -= Eigen::Vector3d(0.0, 1.0, 0.0);
    if (move.squaredNorm() > 1e-8) {
      orbit.target += move.normalized() * move_speed;
      camera_changed = true;
    }

    if (camera_changed && !inflight) {
      request_render(orbit, render_job, inflight);
    }

    if (inflight &&
        render_job.wait_for(std::chrono::milliseconds(0)) ==
            std::future_status::ready) {
      RenderResult res = render_job.get();
      inflight = false;
      if (res.width == width && res.height == height &&
          static_cast<int>(res.pixels.size()) == width * height * 3) {
        latest = std::move(res);
      } else {
        request_render(orbit, render_job, inflight);
      }
    }

    if (!latest.pixels.empty() && texture &&
        latest.width == width && latest.height == height) {
      SDL_UpdateTexture(texture, nullptr, latest.pixels.data(), width * 3);
      SDL_RenderClear(renderer);
      SDL_RenderCopy(renderer, texture, nullptr, nullptr);
      SDL_RenderPresent(renderer);
    } else {
      SDL_SetRenderDrawColor(renderer, 10, 10, 14, 255);
      SDL_RenderClear(renderer);
      SDL_RenderPresent(renderer);
    }
  }

  if (texture)
    SDL_DestroyTexture(texture);
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}
