#include "raylib.h"
#include "rcamera.h"
#include "raymath.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <time.h>

#define NB_BAT 1

#define NB_OISEAUX 10
#define LIMITES 32
#define SPAWN_LIMITES 5

#define MAX_SPEED 0.25
#define NEIGHBOR_RADIUS 10
#define COHESION_FORCE 0.005
#define ALIGN_FORCE 0.03
#define SEPARATION_FORCE 0.1
#define SEPARATION_RADIUS 4

float randomFloat(float min, float max) {
	float range = max - min;
	float random = ((float) rand() / RAND_MAX) * range + min;
	return random;
}

Vector3 randomVector3(double min, double max) {
  return (Vector3){ randomFloat(min, max), randomFloat(min, max), randomFloat(min, max) };
}

typedef struct oiseau {
	int i;
	Vector3 pos;
	Vector3 velo;
} oiseau;

typedef struct nuee {
	oiseau** oiseaux;
	int taille;
} nuee;

nuee nueePrincipale;

oiseau* nouvelOiseau(Vector3 pos, Vector3 velo) {
	oiseau* o = malloc(sizeof(oiseau));

	o->pos = pos;
	o->velo = velo;
	
	return o;
}

Vector3 cohesion(oiseau* o, nuee nuee) {
	Vector3 res = Vector3Zero();

	if(nuee.taille == 0) return res;
	
	for(int i = 0; i < nuee.taille; i++)
		res = Vector3Add(res, nuee.oiseaux[i]->pos);
	
	return Vector3Scale(Vector3Subtract(Vector3Scale(res, 1/nuee.taille), o->pos), COHESION_FORCE);
}

Vector3 alignement(oiseau* o, nuee nuee) {
	Vector3 res = Vector3Zero();

	if(nuee.taille == 0) return res;
	
	for(int i = 0; i < nuee.taille; i++)
		res = Vector3Add(res, nuee.oiseaux[i]->velo);
	
	return Vector3Scale(Vector3Scale(res, 1/nuee.taille), ALIGN_FORCE);
}

Vector3 separation(oiseau* o, nuee nuee) {
	Vector3 res = Vector3Zero();

	for(int i = 0; i < nuee.taille; i++) {
		float d = Vector3Distance(o->pos, nuee.oiseaux[i]->pos);
		if(d < SEPARATION_RADIUS)
			res = Vector3Add(res, Vector3Subtract(o->pos, nuee.oiseaux[i]->pos));
	}

	return Vector3Scale(res, SEPARATION_FORCE);
}

void limite_vitesse(oiseau* o) {
	float v = Vector3Length(o->velo);
	if(v > MAX_SPEED) o->velo = Vector3Scale(o->velo, MAX_SPEED / v);
}

void deplacement(oiseau* o, nuee nuee) {
	Vector3 co = cohesion(o, nuee);
	Vector3 al = alignement(o, nuee);
	Vector3 se = separation(o, nuee);

	o->velo = Vector3Add(o->velo, Vector3Add(co, Vector3Add(al, se)));
	limite_vitesse(o);
	o->pos = Vector3Add(o->pos, o->velo);
}

nuee calculVoisins(oiseau* o) {
	int taille = 0;

	for(int i = 0; i < nueePrincipale.taille; i++) {
		if(nueePrincipale.oiseaux[i]->i != o->i) {
			float d = Vector3Distance(o->pos, nueePrincipale.oiseaux[i]->pos);
			if(d < NEIGHBOR_RADIUS)
				taille++;
		}
	}

	oiseau** tab = malloc(sizeof(oiseau*) * taille);

	int j = 0;
	for(int i = 0; i < nueePrincipale.taille; i++) {
		if(nueePrincipale.oiseaux[i]->i != o->i) {
			float d = Vector3Distance(o->pos, nueePrincipale.oiseaux[i]->pos);
			if(d < NEIGHBOR_RADIUS) {
				tab[j] = nueePrincipale.oiseaux[i];
				j++;
			}
		}
	}

	nuee res = {
		.oiseaux = tab,
		.taille = taille
	};

	return res;
}

void boids(void) {
	for(int i = 0; i < nueePrincipale.taille; i++) {
		nuee voisins = calculVoisins(nueePrincipale.oiseaux[i]);
		deplacement(nueePrincipale.oiseaux[i], voisins);
		free(voisins.oiseaux);
	}
}

void afficheNuee(nuee nuee) {
	for(int i = 0; i < nuee.taille; i++) {
		oiseau o = *nuee.oiseaux[i];
		printf("\n === O#%i ===\n  X = %f\n  Y = %f\n  Z = %f\n  Speed = %f\n", o.i, o.pos.x, o.pos.y, o.pos.z, Vector3Length(o.velo));
	}
}

void freeNuee(nuee nuee) {
  for(int i = 0; i < nuee.taille; i++) free(nuee.oiseaux[i]);
  free(nuee.oiseaux);
}

int main(void) {
	srand(time(NULL));

	const int screenWidth = 1920;
	const int screenHeight = 1080;

	oiseau** t = malloc(sizeof(oiseau*) * NB_OISEAUX);

	for(int i = 0; i < NB_OISEAUX; i++) {
		oiseau* o = malloc(sizeof(oiseau));
		o->i = i;
		o->pos = (Vector3){ (float)GetRandomValue(-SPAWN_LIMITES, SPAWN_LIMITES), 22.0f, (float)GetRandomValue(-SPAWN_LIMITES, SPAWN_LIMITES) };
		o->velo = randomVector3(-MAX_SPEED / 2.f, MAX_SPEED / 2.f);
		t[i] = o;
	}

	nueePrincipale.oiseaux = t;
	nueePrincipale.taille = NB_OISEAUX;

	InitWindow(screenWidth, screenHeight, "TIPE - Simulateur d'Étourmi");
	ToggleBorderlessWindowed();
	ToggleFullscreen();

	Camera camera = { 0 };
	camera.position = (Vector3){ 0.0f, 2.0f, 4.0f };
	camera.target = (Vector3){ 0.0f, 2.0f, 0.0f };
	camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
	camera.fovy = 60.0f;
	camera.projection = CAMERA_PERSPECTIVE;

	Vector3 sizes[NB_BAT] = { 0 };
	Vector3 positions[NB_BAT] = { 0 };
	Color colors[NB_BAT] = { 0 };

	for (int i = 0; i < NB_BAT; i++) {
		sizes[i] = (Vector3){ (float)GetRandomValue(1, 15), (float)GetRandomValue(1, 20), (float)GetRandomValue(1, 15) };
		positions[i] = (Vector3){ (float)GetRandomValue(-32 + sizes[i].x, 32 - sizes[i].x), sizes[i].y/2.0f, (float)GetRandomValue(-32 + sizes[i].z, 32 - sizes[i].z) };
		colors[i] = (Color){ GetRandomValue(20, 255), GetRandomValue(10, 55), 30, 255 };
	}

	bool pause = true;
	bool showRayon = false;
	bool showVitesse = false;

	DisableCursor();

	SetTargetFPS(60);

	while (!WindowShouldClose()) {
		UpdateCamera(&camera, CAMERA_FREE);

		if(IsKeyPressed(KEY_P)) pause = !pause;
		if(IsKeyPressed(KEY_R)) showRayon = !showRayon;
		if(IsKeyPressed(KEY_V)) showVitesse = !showVitesse;

		if(!pause) boids();

		BeginDrawing();{
			ClearBackground(RAYWHITE);

			BeginMode3D(camera);{
				DrawLine3D((Vector3){ 0.0f, 0.0f, 0.0f },(Vector3){ 1.0f, 0.0f, 0.0f }, RED);
				DrawLine3D((Vector3){ 0.0f, 0.0f, 0.0f },(Vector3){ 0.0f, 1.0f, 0.0f }, BLUE);
				DrawLine3D((Vector3){ 0.0f, 0.0f, 0.0f },(Vector3){ 0.0f, 0.0f, 1.0f }, GREEN);

				DrawPlane((Vector3){ 0.0f, 0.0f, 0.0f }, (Vector2){ 64.0f, 64.0f }, LIGHTGRAY);
				DrawCube((Vector3){ -32.0f, 2.5f, 0.0f }, 1.0f, 5.0f, 64.0f, BLACK);
				DrawCube((Vector3){ 32.0f, 2.5f, 0.0f }, 1.0f, 5.0f, 64.0f, BLACK);
				DrawCube((Vector3){ 0.0f, 2.5f, 32.0f }, 64.0f, 5.0f, 1.0f, BLACK);
				DrawCube((Vector3){ 0.0f, 2.5f, -32.0f }, 64.0f, 5.0f, 1.0f, BLACK);

				for (int i = 0; i < NB_BAT; i++) {
					DrawCubeV(positions[i], sizes[i], colors[i]);
					DrawCubeWiresV(positions[i], sizes[i], MAROON);
				}

				// afficheNuee(nueePrincipale);
				for (int i = 0; i < nueePrincipale.taille; i++) {
					DrawSphere(nueePrincipale.oiseaux[i]->pos, 0.5f, GREEN);
					if(showRayon) DrawSphereWires(nueePrincipale.oiseaux[i]->pos, NEIGHBOR_RADIUS, 5, 5, MAROON);
					if(showVitesse) DrawLine3D(nueePrincipale.oiseaux[i]->pos, Vector3Add(nueePrincipale.oiseaux[i]->pos, Vector3Scale(nueePrincipale.oiseaux[i]->velo, 2.f)), RED);
				}
			}EndMode3D();

			DrawFPS(1, 1);
		}EndDrawing();
	}

	CloseWindow();

	freeNuee(nueePrincipale);
	return 0;
}