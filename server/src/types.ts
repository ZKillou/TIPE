export interface PostRequestData {
  // Constantes de la simulation
  g: number;
  dt: number;
  theta: number;
  epsilon: number;
  gfactor: number;

  // Valeurs de la simulation
  ticks: number;
  tries: number;
  bodies: Body[];

  // Résultats de la simulation
  bruteForceTime: number;
  barnesHutTime: number;
  deltaDist: number;
}

export interface Body {
  Pos: Vector3;
  Velo: Vector3;
  Accel: Vector3;
  Mass: number;
};

export interface Vector3 {
  X: number;
  Y: number;
  Z: number;
};
