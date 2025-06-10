#include "raylib.h"
#include "rcamera.h"
#include "raymath.h"
#include "rlgl.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>

// Objets dans l'espace 3D
#define NB_BAT 50
#define NB_OISEAUX 200

// Limites spaciales
#define LIMITES 128

// Apparition des oiseaux
#define SPAWN_LIMITES 32
#define SPAWN_LIMITES_CHAOS 96
#define SPAWN_CHAOS 0

// Limites physiques
#define MAX_SPEED 0.25
#define MAX_ACCEL 0.025
#define MAX_INCLINAISON PI/6
#define MAX_INCLINAISON_TAN tan(MAX_INCLINAISON)
#define AMORTI 0.99

// Voisinage
#define NEIGHBOR_RADIUS 10
#define NEIGHBOR_ANGLE 3*PI/4
#define CELL_SIZE NEIGHBOR_RADIUS
#define GRID_SIZE 8000

// Cohésion
#define COHESION_FORCE 0.005
#define COHESION_MAX_FORCE 0.007

// Alignement
#define ALIGN_FORCE 0.5

// Séparation
#define SEPARATION_FORCE 0.2
#define SEPARATION_RADIUS 4

// Sol
#define MARGE_SOL 1
#define FORCE_SOL 1

// Limites
#define MARGE_LIMITES 2
#define FORCE_LIMITES 0.1
#define LONGUEUR_CARACTERISTIQUE_LIMITES 50
#define LIMITE_PLAFOND 64
#define FORCE_PLAFOND 5

// Collisions
#define FORCE_MAX_BATIMENT 1
#define MARGE_BATIMENT 4
#define LONGUEUR_CARACTERISTIQUE_BATIMENT 50

// Mouvements carte
#define FORCE_BRUIT 0.04
#define FORCE_CIBLE 0.04
#define FORCE_CIBLE_ORBITE 50
#define RADIUS_CIBLE 10
#define VUE_CIBLE 4
#define CIBLE_AWARE 20
#define FORCE_EVICTION 5
#define RADIUS_EVICTION 20
#define LONGUEUR_CARACTERISTIQUE_EVICTION 40
#define EXPLORE_FORCE_PETITE_NUEE 0.005
#define EXPLORE_FORCE_GRANDE_NUEE 0.002

// Raylib+ (https://github.com/raysan5/raylib/blob/master/examples/text/text_draw_3d.c)
static void DrawTextCodepoint3D(Font font, int codepoint, Vector3 position, float fontSize, bool backface, Color tint)
{
  int index = GetGlyphIndex(font, codepoint);
  float scale = fontSize/(float)font.baseSize;

  // Character destination rectangle on screen
  // NOTE: We consider charsPadding on drawing
  position.x += (float)(font.glyphs[index].offsetX - font.glyphPadding)/(float)font.baseSize*scale;
  position.z += (float)(font.glyphs[index].offsetY - font.glyphPadding)/(float)font.baseSize*scale;

  // Character source rectangle from font texture atlas
  // NOTE: We consider chars padding when drawing, it could be required for outline/glow shader effects
  Rectangle srcRec = { font.recs[index].x - (float)font.glyphPadding, font.recs[index].y - (float)font.glyphPadding,
                       font.recs[index].width + 2.0f*font.glyphPadding, font.recs[index].height + 2.0f*font.glyphPadding };

  float width = (float)(font.recs[index].width + 2.0f*font.glyphPadding)/(float)font.baseSize*scale;
  float height = (float)(font.recs[index].height + 2.0f*font.glyphPadding)/(float)font.baseSize*scale;

  if (font.texture.id > 0)
  {
    const float x = 0.0f;
    const float y = 0.0f;
    const float z = 0.0f;

    // normalized texture coordinates of the glyph inside the font texture (0.0f -> 1.0f)
    const float tx = srcRec.x/font.texture.width;
    const float ty = srcRec.y/font.texture.height;
    const float tw = (srcRec.x+srcRec.width)/font.texture.width;
    const float th = (srcRec.y+srcRec.height)/font.texture.height;

    rlCheckRenderBatchLimit(4 + 4*backface);
    rlSetTexture(font.texture.id);

    rlPushMatrix();
      rlTranslatef(position.x, position.y, position.z);

      rlBegin(RL_QUADS);
        rlColor4ub(tint.r, tint.g, tint.b, tint.a);

        // Front Face
        rlNormal3f(0.0f, 1.0f, 0.0f);                                   // Normal Pointing Up
        rlTexCoord2f(tx, ty); rlVertex3f(x,         y, z);              // Top Left Of The Texture and Quad
        rlTexCoord2f(tx, th); rlVertex3f(x,         y, z + height);     // Bottom Left Of The Texture and Quad
        rlTexCoord2f(tw, th); rlVertex3f(x + width, y, z + height);     // Bottom Right Of The Texture and Quad
        rlTexCoord2f(tw, ty); rlVertex3f(x + width, y, z);              // Top Right Of The Texture and Quad

        if (backface)
        {
          // Back Face
          rlNormal3f(0.0f, -1.0f, 0.0f);                              // Normal Pointing Down
          rlTexCoord2f(tx, ty); rlVertex3f(x,         y, z);          // Top Right Of The Texture and Quad
          rlTexCoord2f(tw, ty); rlVertex3f(x + width, y, z);          // Top Left Of The Texture and Quad
          rlTexCoord2f(tw, th); rlVertex3f(x + width, y, z + height); // Bottom Left Of The Texture and Quad
          rlTexCoord2f(tx, th); rlVertex3f(x,         y, z + height); // Bottom Right Of The Texture and Quad
        }
      rlEnd();
    rlPopMatrix();

    rlSetTexture(0);
  }
}

static void DrawText3D(Font font, const char *text, Vector3 position, float fontSize, float fontSpacing, float lineSpacing, bool backface, Color tint)
{
  int length = TextLength(text);          // Total length in bytes of the text, scanned by codepoints in loop

  float textOffsetY = 0.0f;               // Offset between lines (on line break '\n')
  float textOffsetX = 0.0f;               // Offset X to next character to draw

  float scale = fontSize/(float)font.baseSize;

  for (int i = 0; i < length;)
  {
    // Get next codepoint from byte string and glyph index in font
    int codepointByteCount = 0;
    int codepoint = GetCodepoint(&text[i], &codepointByteCount);
    int index = GetGlyphIndex(font, codepoint);

    // NOTE: Normally we exit the decoding sequence as soon as a bad byte is found (and return 0x3f)
    // but we need to draw all of the bad bytes using the '?' symbol moving one byte
    if (codepoint == 0x3f) codepointByteCount = 1;

    if (codepoint == '\n')
    {
      // NOTE: Fixed line spacing of 1.5 line-height
      // TODO: Support custom line spacing defined by user
      textOffsetY += scale + lineSpacing/(float)font.baseSize*scale;
      textOffsetX = 0.0f;
    }
    else
    {
      if ((codepoint != ' ') && (codepoint != '\t'))
      {
        DrawTextCodepoint3D(font, codepoint, (Vector3){ position.x + textOffsetX, position.y, position.z + textOffsetY }, fontSize, backface, tint);
      }

      if (font.glyphs[index].advanceX == 0) textOffsetX += (float)(font.recs[index].width + fontSpacing)/(float)font.baseSize*scale;
      else textOffsetX += (float)(font.glyphs[index].advanceX + fontSpacing)/(float)font.baseSize*scale;
    }

    i += codepointByteCount;   // Move text bytes counter to next codepoint
  }
}

static Vector3 MeasureText3D(Font font, const char* text, float fontSize, float fontSpacing, float lineSpacing)
{
  int len = TextLength(text);
  int tempLen = 0;                // Used to count longer text line num chars
  int lenCounter = 0;

  float tempTextWidth = 0.0f;     // Used to count longer text line width

  float scale = fontSize/(float)font.baseSize;
  float textHeight = scale;
  float textWidth = 0.0f;

  int letter = 0;                 // Current character
  int index = 0;                  // Index position in sprite font

  for (int i = 0; i < len; i++)
  {
    lenCounter++;

    int next = 0;
    letter = GetCodepoint(&text[i], &next);
    index = GetGlyphIndex(font, letter);

    // NOTE: normally we exit the decoding sequence as soon as a bad byte is found (and return 0x3f)
    // but we need to draw all of the bad bytes using the '?' symbol so to not skip any we set next = 1
    if (letter == 0x3f) next = 1;
    i += next - 1;

    if (letter != '\n')
    {
      if (font.glyphs[index].advanceX != 0) textWidth += (font.glyphs[index].advanceX+fontSpacing)/(float)font.baseSize*scale;
      else textWidth += (font.recs[index].width + font.glyphs[index].offsetX)/(float)font.baseSize*scale;
    }
    else
    {
      if (tempTextWidth < textWidth) tempTextWidth = textWidth;
      lenCounter = 0;
      textWidth = 0.0f;
      textHeight += scale + lineSpacing/(float)font.baseSize*scale;
    }

    if (tempLen < lenCounter) tempLen = lenCounter;
  }

  if (tempTextWidth < textWidth) tempTextWidth = textWidth;

  Vector3 vec = { 0 };
  vec.x = tempTextWidth + (float)((tempLen - 1)*fontSpacing/(float)font.baseSize*scale); // Adds chars spacing to measure
  vec.y = 0.25f;
  vec.z = textHeight;

  return vec;
}

// Fonctions utiles
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

// Structures
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

typedef struct ensembleNuee {
  nuee** nuees;
  int taille;
} ensembleNuee;

typedef struct pcellule {
  int x, z;
} pcellule;

typedef struct batiment {
  Vector3 position;
  Vector3 taille;
  Color couleur;
  bool toit;
} batiment;

// Variables globales
nuee nueePrincipale;
batiment batiments[NB_BAT];
nuee grille[GRID_SIZE / 2][GRID_SIZE / 2];
int nbCellules = 0;
bool cibleActivee = true;
bool cibleAware[NB_OISEAUX];
bool pointFuiteActive = false;

// Fonctions nuée
void afficheNuee(nuee nuee) {
  for(int i = 0; i < nuee.taille; i++) {
    if(nuee.oiseaux[i] != NULL) {
      oiseau o = *nuee.oiseaux[i];
      printf("\n === O#%i ===\n  X = %f\n  Y = %f\n  Z = %f\n  Speed = %f\n", o.i, o.pos.x, o.pos.y, o.pos.z, Vector3Length(o.velo));
    }
  }
}

void freeNuee(nuee nuee) {
  for(int i = 0; i < nuee.taille; i++) free(nuee.oiseaux[i]);
  free(nuee.oiseaux);
}

void affichePositionCellule(pcellule pcell) {
  printf("pcell X = %i, Z = %i", pcell.x, pcell.z);
}

// Corps du code
bool estDansVoisinage(oiseau* o1, oiseau* o2, float rayon, float angle) {
  return Vector3Distance(o1->pos, o2->pos) < rayon &&
  Vector3Angle(o1->velo, Vector3Subtract(o2->pos, o1->pos)) <= angle;
}

Vector3 centreDeMasse(nuee nuee) {
  Vector3 res = Vector3Zero();

  if(nuee.taille == 0)
    return res;
  
  for(int i = 0; i < nuee.taille; i++) {
    res = Vector3Add(res, nuee.oiseaux[i]->pos);
  }

  res = Vector3Scale(res, 1.f / nuee.taille);

  return res;
}

Vector3 vitesseLocale(nuee nuee) {
  Vector3 res = Vector3Zero();

  if(nuee.taille == 0)
    return res;
  
  for(int i = 0; i < nuee.taille; i++) {
    res = Vector3Add(res, nuee.oiseaux[i]->velo);
  }

  res = Vector3Scale(res, 1.f / nuee.taille);

  return res;
}

Vector3 cohesion(oiseau* o, nuee nuee) {
  Vector3 res = centreDeMasse(nuee);
  
  res = Vector3Scale(Vector3Subtract(res, o->pos), COHESION_FORCE);

  if (Vector3Length(res) > COHESION_MAX_FORCE)
    res = Vector3Scale(Vector3Normalize(res), COHESION_MAX_FORCE);

  return res;
}

Vector3 alignement(oiseau* o, nuee nuee) {
  return Vector3Scale(vitesseLocale(nuee), ALIGN_FORCE);
}

Vector3 separation(oiseau* o, nuee nuee) {
  Vector3 res = Vector3Zero();

  for(int i = 0; i < nuee.taille; i++) {
    if(estDansVoisinage(o, nuee.oiseaux[i], SEPARATION_RADIUS, NEIGHBOR_ANGLE))
      res = Vector3Add(res, Vector3Subtract(o->pos, nuee.oiseaux[i]->pos));
  }

  return Vector3Scale(res, SEPARATION_FORCE);
}

Vector3 limites(oiseau* o) {
  Vector3 res = Vector3Zero();

  if(o->pos.y < MARGE_SOL) res = Vector3Add(res, (Vector3){ 0.f, FORCE_SOL, 0.f });
  if(o->pos.y > LIMITE_PLAFOND) res = Vector3Add(res, (Vector3){ 0.f, - FORCE_PLAFOND, 0.f });

  if(o->pos.x < - LIMITES + MARGE_LIMITES) res = Vector3Add(res, (Vector3){ FORCE_LIMITES * exp(-fabs(o->pos.x + LIMITES) / LONGUEUR_CARACTERISTIQUE_LIMITES), 0.f, 0.f });
  else if(o->pos.x > LIMITES - MARGE_LIMITES) res = Vector3Add(res, (Vector3){ -FORCE_LIMITES * exp(-fabs(o->pos.x - LIMITES) / LONGUEUR_CARACTERISTIQUE_LIMITES), 0.f, 0.f });
  if(o->pos.z < - LIMITES + MARGE_LIMITES) res = Vector3Add(res, (Vector3){ 0.f, 0.f, FORCE_LIMITES * exp(-fabs(o->pos.z + LIMITES) / LONGUEUR_CARACTERISTIQUE_LIMITES) });
  else if(o->pos.z > LIMITES - MARGE_LIMITES) res = Vector3Add(res, (Vector3){ 0.f, 0.f, -FORCE_LIMITES * exp(-fabs(o->pos.z - LIMITES) / LONGUEUR_CARACTERISTIQUE_LIMITES) });

  return res;
}

void shuffleCibleAware() {
  for(int i = 0; i < NB_OISEAUX; i++) {
    int j = rand() % NB_OISEAUX;
    int tempVal = cibleAware[i];
    cibleAware[i] = cibleAware[j];
    cibleAware[j] = tempVal;
  }
}

void initCibleAware() {
  for(int i = 0; i < NB_OISEAUX; i++) {
    if(i < CIBLE_AWARE) cibleAware[i] = true;
    else cibleAware[i] = false;
  }

  shuffleCibleAware();
}

Vector3 mouvementCarte(oiseau* o, Vector3 cible, Vector3 pointFuite, Vector3 ev) {
  Vector3 res = (Vector3){ randomNoise(), randomNoise(), randomNoise() };

  if(cibleActivee) {
    Vector3 versCible = Vector3Subtract(cible, o->pos);
    float d = Vector3Length(versCible);
    if(d <= RADIUS_CIBLE) {
      Vector3 radial = Vector3Normalize(versCible);
      Vector3 tangentielle = (Vector3){ -radial.z, 0, radial.x };

      res = Vector3Add(res, Vector3Scale(tangentielle, FORCE_CIBLE_ORBITE));
    } else if((d > RADIUS_CIBLE && cibleAware[o->i]) || d <= VUE_CIBLE * RADIUS_CIBLE)
      res = Vector3Add(res, Vector3Scale(versCible, FORCE_CIBLE * (Vector3Length(ev) > 0.1 ? 0.25f : 1.f))); 
  }

  if(pointFuiteActive) {
    Vector3 depuisFuite = Vector3Subtract(o->pos, pointFuite);
    float d = Vector3Length(depuisFuite);

    if(d < RADIUS_EVICTION && d > 0.1f) {
      Vector3 radial = Vector3Normalize(depuisFuite);
      res = Vector3Add(res, Vector3Scale(radial, FORCE_EVICTION * exp(-d / LONGUEUR_CARACTERISTIQUE_EVICTION)));
    }
  }

  return res;
}

Vector3 collision(oiseau* o) {
  Vector3 res = Vector3Zero();

  for(int i = 0; i < NB_BAT; i++) {
    Vector3 vec = Vector3Zero();

    Vector3 diff = Vector3Subtract(o->pos, batiments[i].position);
    float dx = fabs(diff.x) - batiments[i].taille.x / 2;
    float dy = fabs(diff.y) - batiments[i].taille.y / 2;
    float dz = fabs(diff.z) - batiments[i].taille.z / 2;

    if(dx < MARGE_BATIMENT && dy < MARGE_BATIMENT && dz < MARGE_BATIMENT) {
      Vector3 radial = Vector3Normalize(diff);
      Vector3 radial_force = Vector3Scale(radial, FORCE_MAX_BATIMENT * exp(-Vector3Length(diff) / LONGUEUR_CARACTERISTIQUE_BATIMENT));

      Vector3 tangent = (Vector3){ -radial.z, 0, radial.x };
      Vector3 tangent_force = Vector3Scale(Vector3Normalize(tangent), 0.5f * FORCE_MAX_BATIMENT);

      res = Vector3Add(res, Vector3Add(radial_force, tangent_force));
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

void limite_vitesse(oiseau* o, bool angle) {
  float v = Vector3Length(o->velo);
  if(v > MAX_SPEED) o->velo = Vector3Scale(o->velo, MAX_SPEED / v);
  o->velo = Vector3Scale(o->velo, AMORTI);

  if(angle) {
    float rc = sqrt(o->velo.x * o->velo.x + o->velo.z * o->velo.z);
    float inclinaison = atan2(o->velo.y, rc);
    if (fabs(inclinaison) > MAX_INCLINAISON)
      o->velo.y *= MAX_INCLINAISON_TAN / fabs(o->velo.y / rc);
  }
}

void limite_accel(oiseau* o) {
  float a = Vector3Length(o->accel);
  if(a > MAX_ACCEL) o->accel = Vector3Scale(o->accel, MAX_ACCEL / a);
}

void deplacement(oiseau* o, nuee nuee, Vector3 cible, Vector3 pointFuite) {
  Vector3 co = cohesion(o, nuee);
  Vector3 al = alignement(o, nuee);
  Vector3 se = separation(o, nuee);
  Vector3 li = limites(o);
  Vector3 ev = collision(o);
  Vector3 mv = mouvementCarte(o, cible, pointFuite, ev);
  Vector3 ex = explore(o, nuee);

  Vector3 sum = Vector3Add(ev, Vector3Add(ex, Vector3Add(mv, Vector3Add(li, Vector3Add(co, Vector3Add(al, se))))));

  o->accel = sum;
  limite_accel(o);
  o->velo = Vector3Add(o->velo, o->accel);
  limite_vitesse(o, cibleActivee);
  o->pos = Vector3Add(o->pos, o->velo);
}

// Voisinage
pcellule positionCellule(oiseau* o) {
  return (pcellule){ (int) abs(2 * o->pos.x) / CELL_SIZE + 1, (int) abs(2 * o->pos.z) / CELL_SIZE + 1 };
}

Vector2 centreCellule(pcellule cellule) {
  return (Vector2){ (cellule.x - 1) * CELL_SIZE - LIMITES, (cellule.z - 1) * CELL_SIZE - LIMITES };
}

void updateGrille() {
  for(int i = 0; i < nbCellules; i++)
    for(int j = 0; j < nbCellules; j++)
      grille[i][j].taille = 0;
  
  for(int n = 0; n < nueePrincipale.taille; n++) {
    pcellule pcell = positionCellule(nueePrincipale.oiseaux[n]);
    grille[pcell.x][pcell.z].oiseaux[grille[pcell.x][pcell.z].taille++] = nueePrincipale.oiseaux[n];
  }
}

int tailleVoisin(oiseau* o, nuee cell) {
  int taille = 0;

  for(int i = 0; i < cell.taille; i++)
    if(cell.oiseaux[i]->i != o->i && estDansVoisinage(o, cell.oiseaux[i], NEIGHBOR_RADIUS, NEIGHBOR_ANGLE))
      taille++;

  return taille;
}

void ajouteVoisinNuee(oiseau* o, oiseau** t, nuee cell, int* i) {
  for(int j = 0; j < cell.taille; j++) {
    if(cell.oiseaux[j]->i != o->i && estDansVoisinage(o, cell.oiseaux[j], NEIGHBOR_RADIUS, NEIGHBOR_ANGLE)) {
      t[*i] = cell.oiseaux[j];
      (*i)++;
    }
  }
}

nuee calculVoisins(oiseau* o) {
  pcellule pcell = positionCellule(o);

  int taille =
    tailleVoisin(o, grille[pcell.x + 1][pcell.z - 1]) +
    tailleVoisin(o, grille[pcell.x + 1][pcell.z]) +
    tailleVoisin(o, grille[pcell.x + 1][pcell.z + 1]) +
    tailleVoisin(o, grille[pcell.x][pcell.z - 1]) +
    tailleVoisin(o, grille[pcell.x][pcell.z]) +
    tailleVoisin(o, grille[pcell.x][pcell.z + 1]) +
    tailleVoisin(o, grille[pcell.x - 1][pcell.z - 1]) +
    tailleVoisin(o, grille[pcell.x - 1][pcell.z]) +
    tailleVoisin(o, grille[pcell.x - 1][pcell.z + 1]);

  if(!taille) return (nuee){ NULL, 0 };

  oiseau** tab = malloc(sizeof(oiseau*) * taille);

  int i = 0;
  ajouteVoisinNuee(o, tab, grille[pcell.x + 1][pcell.z - 1], &i);
  ajouteVoisinNuee(o, tab, grille[pcell.x + 1][pcell.z], &i);
  ajouteVoisinNuee(o, tab, grille[pcell.x + 1][pcell.z + 1], &i);
  ajouteVoisinNuee(o, tab, grille[pcell.x][pcell.z - 1], &i);
  ajouteVoisinNuee(o, tab, grille[pcell.x][pcell.z], &i);
  ajouteVoisinNuee(o, tab, grille[pcell.x][pcell.z + 1], &i);
  ajouteVoisinNuee(o, tab, grille[pcell.x - 1][pcell.z - 1], &i);
  ajouteVoisinNuee(o, tab, grille[pcell.x - 1][pcell.z], &i);
  ajouteVoisinNuee(o, tab, grille[pcell.x - 1][pcell.z + 1], &i);

  nuee res = (nuee){ tab, taille };

  return res;
}

void parcoursProfondeur(oiseau* o, nuee* groupe, bool* visite) {
  visite[o->i] = true;
  groupe->oiseaux[groupe->taille++] = o;

  nuee voisins = calculVoisins(o);
  for(int i = 0; i < voisins.taille; i++) {
    if(!visite[voisins.oiseaux[i]->i]) {
      parcoursProfondeur(voisins.oiseaux[i], groupe, visite);
    }
  }

  free(voisins.oiseaux);
}

ensembleNuee composantesConnexes() {
  ensembleNuee res = { malloc(NB_OISEAUX * sizeof(nuee)), 0 };
  bool* visite = malloc(NB_OISEAUX * sizeof(bool));
  for(int i = 0; i < NB_OISEAUX; i++) visite[i] = false;

  for(int i = 0; i < nueePrincipale.taille; i++) {
    if(!visite[nueePrincipale.oiseaux[i]->i]) {
      nuee* groupe = malloc(sizeof(nuee*));
      *groupe = (nuee){ malloc(NB_OISEAUX * sizeof(oiseau*)), 0 };
      parcoursProfondeur(nueePrincipale.oiseaux[i], groupe, visite);
      res.nuees[res.taille++] = groupe;
    }
  }

  free(visite);
  return res;
}

// Statistiques
float polarisationLocale(nuee nuee) {
  if(nuee.taille == 0) return 0.f;

  Vector3 sommeVitesses = (Vector3){0.f, 0.f, 0.f};

  for(int i = 0; i < nuee.taille; i++) {
    sommeVitesses = Vector3Add(sommeVitesses, Vector3Normalize(nuee.oiseaux[i]->velo));
  }

  return Vector3Length(sommeVitesses) / (float)nuee.taille;
}

float polarisationLocaleMoyenne() {
  if(nueePrincipale.taille == 0) return 0.f;

  float res = 0.f;

  for(int i = 0; i < nueePrincipale.taille; i++) {
    nuee v = calculVoisins(nueePrincipale.oiseaux[i]);
    res += polarisationLocale(v);
    free(v.oiseaux);
  }

  return res / (float)nueePrincipale.taille;
}

float cohesionLocale(nuee nuee) {
  Vector3 com = centreDeMasse(nuee);

  float res = 0.f;
  for(int i = 0; i < nuee.taille; i++) {
    res += Vector3Distance(com, nuee.oiseaux[i]->pos);
  }

  return res / nuee.taille;
}

float dispertionLocale(nuee nuee) {
  float res = 0.f;

  for (int i = 0; i < nuee.taille; i++) {
    for (int j = i + 1; j < nuee.taille; j++) {
      float d = Vector3Distance(nuee.oiseaux[i]->pos, nuee.oiseaux[j]->pos);
      if (d > res) {
        res = d;
      }
    }
  }

  return res;
}

// Fonction simulation
void boids(Vector3 cible, Vector3 pointFuite) {
  for(int i = 0; i < nueePrincipale.taille; i++) {
    nuee voisins = calculVoisins(nueePrincipale.oiseaux[i]);
    deplacement(nueePrincipale.oiseaux[i], voisins, cible, pointFuite);
    free(voisins.oiseaux);
  }
}

// main
int main(void) {
  srand(time(NULL));

  const int screenWidth = 1440;
  const int screenHeight = 810;

  oiseau** t = malloc(sizeof(oiseau*) * NB_OISEAUX);

  Vector3 vitesseSansChaos = randomVector3(-MAX_SPEED / 2.f, MAX_SPEED / 2.f);
  for(int i = 0; i < NB_OISEAUX; i++) {
    oiseau* o = malloc(sizeof(oiseau));
    o->i = i;
    o->pos = SPAWN_CHAOS == 0 ?
      (Vector3){ (float)GetRandomValue(-SPAWN_LIMITES, SPAWN_LIMITES), 22.0f, (float)GetRandomValue(-SPAWN_LIMITES, SPAWN_LIMITES) } :
      (Vector3){ (float)GetRandomValue(-SPAWN_LIMITES_CHAOS, SPAWN_LIMITES_CHAOS), 22.0f, (float)GetRandomValue(-SPAWN_LIMITES_CHAOS, SPAWN_LIMITES_CHAOS) };
    o->velo = SPAWN_CHAOS == 0 ?
      vitesseSansChaos :
      randomVector3(-MAX_SPEED / 2.f, MAX_SPEED / 2.f);
    t[i] = o;
  }

  nueePrincipale.oiseaux = t;
  nueePrincipale.taille = NB_OISEAUX;

  for(int i = 0; i < 2 * LIMITES / 2 + 3; i++) {
    for(int j = 0; j < 2 * LIMITES / 2 + 3; j++) {
      oiseau** oiseaux = malloc(NB_OISEAUX * sizeof(oiseau*));
      nuee cell = (nuee){ oiseaux, 0 };
      grille[i][j] = cell;
    }
    nbCellules++;
  }

  InitWindow(screenWidth, screenHeight, "TIPE - Simulateur d'Étourmi");

  Camera camera = { 0 };
  camera.position = (Vector3){ 0.0f, 1.5f * LIMITE_PLAFOND, 4.0f };
  camera.target = (Vector3){ 0.0f, 2.0f, 0.0f };
  camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
  camera.fovy = 120.0f;
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
  bool showPolarisation = false;
  bool showNueesStats = false;

  Vector3 cible = randomCible();
  Vector3 pointFuite = randomCible();
  initCibleAware();
  updateGrille();

  Font defaultFont = GetFontDefault();

  DisableCursor();

  SetTargetFPS(60);

  while (!WindowShouldClose()) {
    UpdateCamera(&camera, CAMERA_FREE);

    if(IsKeyPressed(KEY_P)) pause = !pause;
    if(IsKeyPressed(KEY_R)) showRayon = !showRayon;
    if(IsKeyPressed(KEY_V)) showVitesse = !showVitesse;
    if(IsKeyPressed(KEY_C) && !IsKeyDown(KEY_LEFT_SHIFT)) cible = randomCible();
    if(IsKeyPressed(KEY_C) && IsKeyDown(KEY_LEFT_SHIFT)) cibleActivee = !cibleActivee;
    if(IsKeyPressed(KEY_F) && !IsKeyDown(KEY_LEFT_SHIFT)) {
      cibleActivee = false;
      pointFuite = cible;
      pointFuiteActive = true;
    }
    if(IsKeyPressed(KEY_F) && IsKeyDown(KEY_LEFT_SHIFT)) pointFuiteActive = false;
    if(IsKeyPressed(KEY_H)) showPolarisation = !showPolarisation;
    if(IsKeyPressed(KEY_N)) showNueesStats = !showNueesStats;
    if(IsKeyPressed(KEY_T)) shuffleCibleAware();

    if(!pause) {
      updateGrille();
      boids(cible, pointFuite);
    }

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
        if(pointFuiteActive) DrawCube(pointFuite, 1, 1, 1, BLACK);

        for (int i = 0; i < nueePrincipale.taille; i++) {
          DrawSphere(nueePrincipale.oiseaux[i]->pos, 0.125f, cibleAware[i] && cibleActivee ? LIME : DARKBLUE);
          if(showRayon) DrawSphereWires(nueePrincipale.oiseaux[i]->pos, NEIGHBOR_RADIUS, 5, 5, MAROON);
          if(showVitesse) DrawLine3D(nueePrincipale.oiseaux[i]->pos, Vector3Add(nueePrincipale.oiseaux[i]->pos, Vector3Scale(nueePrincipale.oiseaux[i]->velo, 2.f)), RED);
        }

        if(showNueesStats) {
          ensembleNuee ensemble = composantesConnexes();
          for(int i = 0; i < ensemble.taille; i++) {
            Vector3 com = centreDeMasse(*ensemble.nuees[i]);
            com.y = LIMITE_PLAFOND + 1;

            const char* text = TextFormat(
              "Taille %i O\nPolarisation %f %%\nCohésion %f m\nDispertion %f m\nVitesse %f m/s",
              ensemble.nuees[i]->taille,
              polarisationLocale(*ensemble.nuees[i]) * 100.f,
              cohesionLocale(*ensemble.nuees[i]),
              dispertionLocale(*ensemble.nuees[i]),
              Vector3Length(vitesseLocale(*ensemble.nuees[i]))
            );

            Vector3 mes = MeasureText3D(defaultFont, text, 24, 2, 2);
            mes.y = 0;

            DrawText3D(defaultFont, text, Vector3Subtract(com, Vector3Scale(mes, 0.5f)), 24, 2, 2, true, BLACK);
          }
        }
      }EndMode3D();

      DrawFPS(1, 1);
      if(showPolarisation) DrawText(TextFormat("Polarisation %f", polarisationLocaleMoyenne()), 1, 21, 20, BLACK);
    }EndDrawing();
  }

  CloseWindow();

  freeNuee(nueePrincipale);
  return 0;
}