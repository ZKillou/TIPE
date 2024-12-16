#include "raylib.h"
#include "rcamera.h"
#include "raymath.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>

#define NB_BAT 20

#define NB_OISEAUX 500
#define LIMITES 128
#define SPAWN_LIMITES 64

#define MAX_SPEED 0.25
#define MAX_ACCEL 0.025
#define AMORTI 0.99

#define NEIGHBOR_RADIUS 10
#define COHESION_FORCE 0.002
#define ALIGN_FORCE 0.003
#define SEPARATION_FORCE 0.005
#define SEPARATION_RADIUS 4

#define FORCE_SOL 0.3
#define FORCE_LIMITES 0.1
#define LIMITE_PLAFOND 64
#define FORCE_PLAFOND 5
#define MARGE_SOL 5
#define MARGE_LIMITES 2
#define LONGUEUR_CARACTERISTIQUE_LIMITES 50
#define FORCE_BRUIT 0.08
#define FORCE_CIBLE 0.005
#define FORCE_MAX_BATIMENT 0.1
#define MARGE_BATIMENT 5
#define LONGUEUR_CARACTERISTIQUE_BATIMENT 20
#define EXPLORE_FORCE_PETITE_NUEE 0.005
#define EXPLORE_FORCE_GRANDE_NUEE 0.002

float randomFloat(float min, float max) {
	float range = max - min;
	float random = ((float) rand() / RAND_MAX) * range + min;
	return random;
}

Vector3 randomVector3(double min, double max) {
  return (Vector3){ randomFloat(min, max), randomFloat(min, max), randomFloat(min, max) };
}

float randomNoise(void) {
	return ((float) rand() / RAND_MAX - 0.5f) * FORCE_BRUIT;
}

Vector3 randomCible(void) {
	return (Vector3){ (float)GetRandomValue(-LIMITES + 1, LIMITES - 1), (float)GetRandomValue(3, 15), (float)GetRandomValue(-LIMITES + 1, LIMITES - 1) };
}

typedef struct oiseau {
	int i;
	Vector3 pos;
	Vector3 velo;
	Vector3 accel;
} oiseau;

typedef struct nuee {
	oiseau** oiseaux;
	int taille;
} nuee;

typedef struct batiment {
	Vector3 position;
	Vector3 taille;
	Color couleur;
	bool toit;
} batiment;

nuee nueePrincipale;
batiment batiments[NB_BAT];
bool cibleActivee = true;

oiseau* nouvelOiseau(Vector3 pos, Vector3 velo) {
	oiseau* o = malloc(sizeof(oiseau));

	o->pos = pos;
	o->velo = velo;
	o->accel = Vector3Zero();
	
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

Vector3 limites(oiseau* o) {
	Vector3 res = Vector3Zero();

	if(o->pos.y < MARGE_SOL) res = Vector3Add(res, (Vector3){ 0.f, fmin((1 / pow(o->pos.y - MARGE_SOL, 2)), FORCE_SOL), 0.f });
	if(o->pos.y > LIMITE_PLAFOND) res = Vector3Add(res, (Vector3){ 0.f, - FORCE_PLAFOND, 0.f });

	if(o->pos.x < - LIMITES + MARGE_LIMITES) res = Vector3Add(res, (Vector3){ FORCE_LIMITES * exp(-fabs(o->pos.x + LIMITES) / LONGUEUR_CARACTERISTIQUE_LIMITES), 0.f, 0.f });
	else if(o->pos.x > LIMITES - MARGE_LIMITES) res = Vector3Add(res, (Vector3){ -FORCE_LIMITES * exp(-fabs(o->pos.x - LIMITES) / LONGUEUR_CARACTERISTIQUE_LIMITES), 0.f, 0.f });
	if(o->pos.z < - LIMITES + MARGE_LIMITES) res = Vector3Add(res, (Vector3){ 0.f, 0.f, FORCE_LIMITES * exp(-fabs(o->pos.z + LIMITES) / LONGUEUR_CARACTERISTIQUE_LIMITES) });
	else if(o->pos.z > LIMITES - MARGE_LIMITES) res = Vector3Add(res, (Vector3){ 0.f, 0.f, -FORCE_LIMITES * exp(-fabs(o->pos.z - LIMITES) / LONGUEUR_CARACTERISTIQUE_LIMITES) });

	return res;
}

Vector3 mouvementCarte(oiseau* o, Vector3 cible) {
	return Vector3Add(
		(Vector3){ randomNoise(), randomNoise(), randomNoise() },
		cibleActivee ? Vector3Scale(Vector3Subtract(cible, o->pos), FORCE_CIBLE) : Vector3Zero()
	);
}

Vector3 collision(oiseau* o) {
	Vector3 res = Vector3Zero();

	for(int i = 0; i < NB_BAT; i++) {
		Vector3 vec = Vector3Zero();

		float gauche = batiments[i].position.x - batiments[i].taille.x / 2;
		float droite = batiments[i].position.x + batiments[i].taille.x / 2;

		float haut = batiments[i].position.y + batiments[i].taille.y / 2;

		float avant = batiments[i].position.z - batiments[i].taille.z / 2;
		float arriere = batiments[i].position.z + batiments[i].taille.z / 2;

		if(o->pos.x > gauche - MARGE_BATIMENT && o->pos.x < gauche) {
			float dist = fabs(o->pos.x - gauche);
			if(dist < MARGE_BATIMENT) {
				float avoid_force = FORCE_MAX_BATIMENT * exp(-dist / LONGUEUR_CARACTERISTIQUE_BATIMENT);
				vec = Vector3Add(vec, (Vector3){ -avoid_force, 0, 0 });
			}
		}
		if(o->pos.x < droite + MARGE_BATIMENT && o->pos.x > droite) {
			float dist = fabs(o->pos.x - droite);
			if(dist < MARGE_BATIMENT) {
				float avoid_force = FORCE_MAX_BATIMENT * exp(-dist / LONGUEUR_CARACTERISTIQUE_BATIMENT);
				vec = Vector3Add(vec, (Vector3){ avoid_force, 0, 0 });
			}
		}

		if(o->pos.y < haut + MARGE_BATIMENT && o->pos.y > haut) {
			float dist = fabs(o->pos.y - haut);
			if(dist < MARGE_BATIMENT) {
				float avoid_force = FORCE_MAX_BATIMENT * exp(-dist / LONGUEUR_CARACTERISTIQUE_BATIMENT);
				vec = Vector3Add(vec, (Vector3){ 0, avoid_force, 0 });
			}
		}

		if(o->pos.z > avant - MARGE_BATIMENT && o->pos.z < avant) {
			float dist = fabs(o->pos.z - avant);
			if(dist < MARGE_BATIMENT) {
				float avoid_force = FORCE_MAX_BATIMENT * exp(-dist / LONGUEUR_CARACTERISTIQUE_BATIMENT);
				vec = Vector3Add(vec, (Vector3){ 0, 0, -avoid_force });
			}
		}
		if(o->pos.z < arriere + MARGE_BATIMENT && o->pos.z > arriere) {
			float dist = fabs(o->pos.z - arriere);
			if(dist < MARGE_BATIMENT) {
				float avoid_force = FORCE_MAX_BATIMENT * exp(-dist / LONGUEUR_CARACTERISTIQUE_BATIMENT);
				vec = Vector3Add(vec, (Vector3){ 0, 0, avoid_force });
			}
		}

		if(Vector3Distance(o->pos, batiments[i].position) / 2 < MARGE_BATIMENT) vec = Vector3Scale(vec, 20.f);

		res = Vector3Add(res, vec);
	}

	float t = Vector3Length(res);
	if(t > FORCE_MAX_BATIMENT) res = Vector3Scale(res, FORCE_MAX_BATIMENT / t);

	if(t < 0.01) return Vector3Zero();

	return res;
}

Vector3 explore(oiseau* o, nuee nuee) {
	return Vector3Scale((Vector3){ randomNoise(), randomNoise(), randomNoise() }, nuee.taille <= 5 ? EXPLORE_FORCE_PETITE_NUEE : EXPLORE_FORCE_GRANDE_NUEE);
}

void limite_vitesse(oiseau* o) {
	float v = Vector3Length(o->velo);
	if(v > MAX_SPEED) o->velo = Vector3Scale(o->velo, MAX_SPEED / v);
	o->velo = Vector3Scale(o->velo, AMORTI);
}

void limite_accel(oiseau* o) {
	float a = Vector3Length(o->accel);
	if(a > MAX_ACCEL) o->accel = Vector3Scale(o->accel, MAX_ACCEL / a);
}

void deplacement(oiseau* o, nuee nuee, Vector3 cible) {
	Vector3 co = cohesion(o, nuee);
	Vector3 al = alignement(o, nuee);
	Vector3 se = separation(o, nuee);
	Vector3 li = limites(o);
	Vector3 mv = mouvementCarte(o, cible);
	Vector3 ev = collision(o);
	Vector3 ex = explore(o, nuee);

	o->accel = Vector3Add(ex, Vector3Add(ev, Vector3Add(mv, Vector3Add(li, Vector3Add(co, Vector3Add(al, se))))));
	limite_accel(o);
	o->velo = Vector3Add(o->velo, o->accel);
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

	if(!taille) return (nuee){ NULL, 0 };

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

void boids(Vector3 cible) {
	for(int i = 0; i < nueePrincipale.taille; i++) {
		nuee voisins = calculVoisins(nueePrincipale.oiseaux[i]);
		// printf("%i a %i voisins\n", i, voisins.taille);
		deplacement(nueePrincipale.oiseaux[i], voisins, cible);
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

	const int screenWidth = 1440;
	const int screenHeight = 810;

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
	// ToggleBorderlessWindowed();
	// ToggleFullscreen();

	Camera camera = { 0 };
	camera.position = (Vector3){ 0.0f, 2.0f, 4.0f };
	camera.target = (Vector3){ 0.0f, 2.0f, 0.0f };
	camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
	camera.fovy = 60.0f;
	camera.projection = CAMERA_PERSPECTIVE;

	for (int i = 0; i < NB_BAT; i++) {
		batiments[i] = (batiment){
			Vector3Zero(),
			(Vector3){ (float)GetRandomValue(1, 15), (float)GetRandomValue(1, 20), (float)GetRandomValue(1, 15) },
			(Color){ GetRandomValue(20, 255), GetRandomValue(10, 55), 30, 255 },
			true
		};
		batiments[i].position = (Vector3){ (float)GetRandomValue(-LIMITES + batiments[i].taille.x, LIMITES - batiments[i].taille.x), batiments[i].taille.y/2.0f, (float)GetRandomValue(-LIMITES + batiments[i].taille.z, LIMITES - batiments[i].taille.z) };
	}

	bool pause = true;
	bool showRayon = false;
	bool showVitesse = false;

	Vector3 cible = randomCible();

	DisableCursor();

	SetTargetFPS(60);

	while (!WindowShouldClose()) {
		UpdateCamera(&camera, CAMERA_FREE);

		if(IsKeyPressed(KEY_P)) pause = !pause;
		if(IsKeyPressed(KEY_R)) showRayon = !showRayon;
		if(IsKeyPressed(KEY_V)) showVitesse = !showVitesse;
		if(IsKeyPressed(KEY_C) && !IsKeyDown(KEY_LEFT_SHIFT)) cible = randomCible();
		if(IsKeyPressed(KEY_C) && IsKeyDown(KEY_LEFT_SHIFT)) cibleActivee = !cibleActivee;

		if(!pause) boids(cible);

		BeginDrawing();{
			ClearBackground(RAYWHITE);

			BeginMode3D(camera);{
				DrawLine3D((Vector3){ 0.0f, 0.0f, 0.0f },(Vector3){ 1.0f, 0.0f, 0.0f }, RED);
				DrawLine3D((Vector3){ 0.0f, 0.0f, 0.0f },(Vector3){ 0.0f, 1.0f, 0.0f }, BLUE);
				DrawLine3D((Vector3){ 0.0f, 0.0f, 0.0f },(Vector3){ 0.0f, 0.0f, 1.0f }, GREEN);

				DrawPlane((Vector3){ 0.0f, 0.0f, 0.0f }, (Vector2){ LIMITES*2, LIMITES*2 }, LIGHTGRAY);
				DrawCube((Vector3){ -LIMITES, 2.5f, 0.0f }, 1.0f, 5.0f, LIMITES*2, BLACK);
				DrawCube((Vector3){ LIMITES, 2.5f, 0.0f }, 1.0f, 5.0f, LIMITES*2, BLACK);
				DrawCube((Vector3){ 0.0f, 2.5f, LIMITES }, LIMITES*2, 5.0f, 1.0f, BLACK);
				DrawCube((Vector3){ 0.0f, 2.5f, -LIMITES }, LIMITES*2, 5.0f, 1.0f, BLACK);

				for (int i = 0; i < NB_BAT; i++) {
					DrawCubeV(batiments[i].position, batiments[i].taille, batiments[i].couleur);
					DrawCubeWiresV(batiments[i].position, batiments[i].taille, MAROON);
				}

				if(cibleActivee) DrawCube(cible, 1, 1, 1, WHITE);

				// afficheNuee(nueePrincipale);
				for (int i = 0; i < nueePrincipale.taille; i++) {
					DrawSphere(nueePrincipale.oiseaux[i]->pos, 0.125f, GREEN);
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