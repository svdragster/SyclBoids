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
#include "boids.hpp"
#include <string>
#include <cstdio>
#include <chrono>

#include <SDL2/SDL.h>
#include <array>
#include <iostream>


using namespace std::chrono;


const size_t frameAmount = 1000;

void render_boids(SDL_Renderer *renderer, std::vector <Boid> boids) {
    SDL_SetRenderDrawColor(renderer, 0x00, 0x00, 0x00, 0xFF);
    SDL_RenderClear(renderer);

    for (Boid boid: boids) {
        SDL_SetRenderDrawColor(renderer, boid.r, boid.g, boid.b, 255);
        SDL_RenderDrawPoint(renderer, (int) boid.x+1, (int) boid.y+1);
        SDL_RenderDrawPoint(renderer, (int) boid.x, (int) boid.y+1);
        SDL_RenderDrawPoint(renderer, (int) boid.x+1, (int) boid.y);
        SDL_RenderDrawPoint(renderer, (int) boid.x, (int) boid.y);
    }

    SDL_RenderPresent(renderer);
    SDL_Delay(5);
}

int main(int argc, char **argv) {

    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        printf("SDL could not initialize! SDL_Error: %s\n", SDL_GetError());
        return 1;
    }

    const size_t width = 1920;
    const size_t height = 1080;

    SDL_Window *window = SDL_CreateWindow("Simple SDL2 Example",
                                          SDL_WINDOWPOS_UNDEFINED,
                                          SDL_WINDOWPOS_UNDEFINED,
                                          width, height,
                                          SDL_WINDOW_SHOWN);


    if (window == nullptr) {
        printf("Window could not be created! SDL_Error: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    // Create a renderer for the window
    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (renderer == nullptr) {
        printf("Renderer could not be created! SDL_Error: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    SDL_Surface *window_surface = SDL_GetWindowSurface(window);

    SDL_SetRenderDrawColor(renderer, 0x00, 0x00, 0x00, 0xFF);

    // Clear the screen with the background color
    SDL_RenderClear(renderer);

    // Update the screen
    SDL_RenderPresent(renderer);

    const bool cpu = false;

    const size_t amount = cpu ? 2500 : 15000;

    std::vector <Boid> boids = Boids::create_boids(width, height, amount);

    //sycl::gpu_selector selector;
    sycl::cpu_selector selector;
    sycl::queue queue(selector);


    bool quit = false;

    SDL_Event e;
    while (!quit) {
        while (!quit && SDL_PollEvent(&e)) {
            switch (e.type) {
                case SDL_KEYDOWN:
                    if (e.key.keysym.sym == SDLK_q) {
                        quit = true;
                    }
                    break;
                case SDL_QUIT:
                    quit = true;
                    break;
            }
        }
        auto start_time = high_resolution_clock::now();
        if (cpu) {
            //boids = BoidsCPU::update_boids_single_thread(boids);
            boids = BoidsCPU::update_boids_openmp(boids);
        } else {
            boids = BoidsSycl::update_boids(queue, boids);
        }
        auto end_time = high_resolution_clock::now();
        auto elapsed_time = duration_cast<milliseconds>(end_time - start_time);

        std::cout << "Elapsed time: " << elapsed_time.count() << " milliseconds\n";
        render_boids(renderer, boids);
    }



    //render_on_surfaces(width, height, window, window_surface, frames);

    //render_frames_directly(width, height, renderer, frames);

    // Clean up SDL
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}

