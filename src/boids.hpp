/*
 *  Boids - a Boids application with SyCL
 *  Copyright (C) 2024  Sven Vollmar & David Schwarzbeck

 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.

 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef BOIDS_BOIDS_HPP
#define BOIDS_BOIDS_HPP


#include <math.h>
#include <vector>
#include <utility>
#include <iostream>
#include <random>
#include <hipSYCL/sycl/libkernel/builtins.hpp>
#include <sycl/sycl.hpp>

const double MAX_RANGE = 40.0;
const double VISIBLE_RANGE_SQUARED = 35.0 * 35.0;
const double CLOSE_RANGE_SQUARED = 3.5 * 3.5;
const double CENTERING_FACTOR = 0.0;
const double MATCHING_FACTOR = 0.05;
const double AVOID_FACTOR = 0.1;
const double TURN_FACTOR = 0.2;
const double MIN_SPEED = 1.0;
const double MAX_SPEED = 1.6q;

class Boid {
public:
    Boid() {

    }

    ~Boid() {

    }

    bool operator==(const Boid &rhs) const {
        return this->id == rhs.id;
    }

    bool operator!=(const Boid &rhs) const {
        return this->id != rhs.id;
    }

    int id;
    double x;
    double y;
    double velocity_x;
    double velocity_y;
    unsigned char r;
    unsigned char g;
    unsigned char b;
};

namespace Boids {

    std::vector <Boid> create_boids(size_t width, size_t height, size_t amount) {
        std::random_device rd;
        std::mt19937 generator(rd());
        std::uniform_int_distribution<int> distribution_x(100, width - 100);
        std::uniform_int_distribution<int> distribution_y(100, height - 100);
        std::uniform_int_distribution<int> distribution_rgb(0, 255);
        std::vector <Boid> boids;
        for (int i = 0; i < amount; i++) {
            Boid boid;
            boid.id = i;
            boid.x = (double) distribution_x(generator);
            boid.y = (double) distribution_y(generator);
            if (distribution_rgb(generator) > 127) {
                boid.r = 255;
                boid.g = 0;
            } else {
                boid.r = 0;
                boid.g = 255;
            }
            boid.b = 127;
            boid.velocity_x = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) * 2 - 1;
            boid.velocity_y = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) * 2 - 1;
            boids.push_back(boid);
        }
        return boids;
    }

    Boid update_boid(Boid &boid, const std::vector <Boid> other_boids) {
        double x_pos_avg = 0;
        double y_pos_avg = 0;
        double x_vel_avg = 0;
        double y_vel_avg = 0;
        unsigned int near_boids_amount = 0;
        double close_dx = 0;
        double close_dy = 0;
        for (Boid other: other_boids) {
            if (other == boid) {
                continue;
            }
            double dx = boid.x - other.x;
            double dy = boid.y - other.y;
            if (abs(dx) < MAX_RANGE && abs(dy) < MAX_RANGE) {
                double distance_squared = dx * dx + dy * dy;
                if (distance_squared < CLOSE_RANGE_SQUARED) {
                    close_dx += boid.x - other.x;
                    close_dy += boid.y - other.y;
                } else if (distance_squared < VISIBLE_RANGE_SQUARED) {
                    x_pos_avg += other.x;
                    y_pos_avg += other.y;
                    x_vel_avg += other.velocity_x;
                    y_vel_avg += other.velocity_y;
                    near_boids_amount += 1;
                }
            }
        }

        if (near_boids_amount > 0) {
            x_pos_avg /= near_boids_amount;
            y_pos_avg /= near_boids_amount;
            x_vel_avg /= near_boids_amount;
            y_vel_avg /= near_boids_amount;

            boid.velocity_x = (boid.velocity_x + (x_pos_avg - boid.x) * CENTERING_FACTOR +
                               (x_vel_avg - boid.velocity_x) * MATCHING_FACTOR);
            boid.velocity_y = (boid.velocity_y + (y_pos_avg - boid.y) * CENTERING_FACTOR +
                               (y_vel_avg - boid.velocity_y) * MATCHING_FACTOR);

        }

        // Avoid close boids
        boid.velocity_x += (close_dx * AVOID_FACTOR);
        boid.velocity_y += (close_dy * AVOID_FACTOR);

        // Avoid screen edges
        if (boid.x < 50) {
            boid.velocity_x += TURN_FACTOR;
        }
        if (boid.x > 1870) {
            boid.velocity_x -= TURN_FACTOR;
        }
        if (boid.y < 50) {
            boid.velocity_y += TURN_FACTOR;
        }
        if (boid.y > 1030) {
            boid.velocity_y -= TURN_FACTOR;
        }


        double speed = sqrt(boid.velocity_x * boid.velocity_x + boid.velocity_y * boid.velocity_y);
        if (speed < MIN_SPEED) {
            boid.velocity_x = (boid.velocity_x / speed) * MIN_SPEED;
            boid.velocity_y = (boid.velocity_y / speed) * MIN_SPEED;
        } else if (speed > MAX_SPEED) {
            boid.velocity_x = (boid.velocity_x / speed) * MAX_SPEED;
            boid.velocity_y = (boid.velocity_y / speed) * MAX_SPEED;
        }

        if (boid.velocity_x > 0) {
            boid.r = std::min(255, (int) (boid.r * 1.01 + 1));
            boid.g = std::max(0, (int) (boid.g * 0.99 - 1));
        } else {
            boid.g = std::min(255, (int) (boid.g * 1.01 + 1));
            boid.r = std::max(0, (int) (boid.r * 0.99 - 1));
        }

        boid.x += boid.velocity_x;
        boid.y += boid.velocity_y;
        return boid;
    }

    Boid update_boid_sycl(Boid &boid,
                          const sycl::accessor<Boid, 1, sycl::access::mode::read, sycl::access::target::global_buffer> &other_boids) {
        double x_pos_avg = 0;
        double y_pos_avg = 0;
        double x_vel_avg = 0;
        double y_vel_avg = 0;
        unsigned int near_boids_amount = 0;
        double close_dx = 0;
        double close_dy = 0;
        for (Boid other: other_boids) {
            if (other == boid) {
                continue;
            }
            double dx = boid.x - other.x;
            double dy = boid.y - other.y;
            if (abs(dx) < MAX_RANGE && abs(dy) < MAX_RANGE) {
                double distance_squared = dx * dx + dy * dy;
                if (distance_squared < CLOSE_RANGE_SQUARED) {
                    close_dx += boid.x - other.x;
                    close_dy += boid.y - other.y;
                } else if (distance_squared < VISIBLE_RANGE_SQUARED) {
                    x_pos_avg += other.x;
                    y_pos_avg += other.y;
                    x_vel_avg += other.velocity_x;
                    y_vel_avg += other.velocity_y;
                    near_boids_amount += 1;
                }
            }
        }

        if (near_boids_amount > 0) {
            x_pos_avg /= near_boids_amount;
            y_pos_avg /= near_boids_amount;
            x_vel_avg /= near_boids_amount;
            y_vel_avg /= near_boids_amount;

            boid.velocity_x = (boid.velocity_x + (x_pos_avg - boid.x) * CENTERING_FACTOR +
                               (x_vel_avg - boid.velocity_x) * MATCHING_FACTOR);
            boid.velocity_y = (boid.velocity_y + (y_pos_avg - boid.y) * CENTERING_FACTOR +
                               (y_vel_avg - boid.velocity_y) * MATCHING_FACTOR);
        }

        // Avoid close boids
        boid.velocity_x += (close_dx * AVOID_FACTOR);
        boid.velocity_y += (close_dy * AVOID_FACTOR);

        // Avoid screen edges
        if (boid.x < 50) {
            boid.velocity_x += TURN_FACTOR;
        }
        if (boid.x > 1870) {
            boid.velocity_x -= TURN_FACTOR;
        }
        if (boid.y < 50) {
            boid.velocity_y += TURN_FACTOR;
        }
        if (boid.y > 1030) {
            boid.velocity_y -= TURN_FACTOR;
        }


        double speed = sqrt(boid.velocity_x * boid.velocity_x + boid.velocity_y * boid.velocity_y);
        if (speed < MIN_SPEED) {
            boid.velocity_x = (boid.velocity_x / speed) * MIN_SPEED;
            boid.velocity_y = (boid.velocity_y / speed) * MIN_SPEED;
        } else if (speed > MAX_SPEED) {
            boid.velocity_x = (boid.velocity_x / speed) * MAX_SPEED;
            boid.velocity_y = (boid.velocity_y / speed) * MAX_SPEED;
        }

        if (boid.velocity_x > 0) {
            boid.r = std::min(255, (int) (boid.r * 1.01 + 1));
            boid.g = std::max(0, (int) (boid.g * 0.99 - 1));
        } else {
            boid.g = std::min(255, (int) (boid.g * 1.01 + 1));
            boid.r = std::max(0, (int) (boid.r * 0.99 - 1));
        }

        boid.x += boid.velocity_x;
        boid.y += boid.velocity_y;
        return boid;
    }
}


namespace BoidsCPU {
    std::vector <Boid> update_boids_single_thread(std::vector <Boid> &boids) {
        for (int i = 0; i < boids.size(); i++) {
            Boid boid = boids[i];
            boids[i] = Boids::update_boid(boid, boids);
        }
        return boids;
    }

    std::vector <Boid> update_boids_openmp(std::vector <Boid> &boids) {
#pragma omp parallel for
        for (int i = 0; i < boids.size(); i++) {
            Boid boid = boids[i];
            boids[i] = Boids::update_boid(boid, boids);
        }
        return boids;
    }
}

namespace BoidsSycl {
    std::vector <Boid> update_boids(sycl::queue queue, std::vector <Boid> &boids) {
        std::vector new_boids = boids;


        sycl::buffer<Boid, 1> bufferBoids(new_boids.data(), sycl::range<1>(new_boids.size()));

        queue.submit([&](sycl::handler &cgh) {
            auto write_access_boids = bufferBoids.get_access<sycl::access::mode::write>(cgh);
            auto read_access_boids = bufferBoids.get_access<sycl::access::mode::read>(cgh);
            cgh.parallel_for<class GameOfLifeLoop>(
                    sycl::range<1>(new_boids.size()),
                    [=](sycl::id<1> idx) {
                        const size_t index = idx[0];
                        Boid boid = write_access_boids[index];
                        write_access_boids[index] = Boids::update_boid_sycl(boid, read_access_boids);
                    }
            );
        });
        queue.wait();
        return new_boids;
    }
}

#endif //BOIDS_BOIDS_HPP
