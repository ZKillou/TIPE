Soit une famille d'oiseaux $E$ de taille $N$. On note $\vec{p_i}$ la position et $\vec{v_i}$ la vitesse du i<sup>ème</sup> oiseau.

### Calcul de la cohésion $\vec{co}$

$$
\frac{\vec{co_i}}{C_{\vec{co}}} = (\frac{1}{N - 1} \sum_{\substack{k = 1 \\ k \neq i}}^{N} \vec{p_{k}}) - \vec{p_i}
$$

### Calcul de l'alignement $\vec{al}$

$$
\frac{\vec{al_i}}{C_{\vec{al}}} = \frac{1}{N - 1} \sum_{\substack{k = 1 \\ k \neq i}}^{N} \vec{v_{k}}
$$

### Calcul de la séparation $\vec{se}$

$$
\frac{\vec{se_i}}{C_{\vec{se}}} = \sum_{\substack{k = 1 \\ k \neq i}}^{N} (\vec{p_{i}} - \vec{p_{k}})
$$

### Calcul de polarisation $\Phi$

$$
\Phi = \lVert \frac{1}{N} \sum_{k = 1}^{N} \frac{\vec{v_i}}{\lVert \vec{v_i} \rVert} \rVert
$$

### Calcul de la cohésion locale $co$

$$
\vec{CoM} = \frac{1}{N} \sum_{k = 1}^{N} \vec{p_i} \\
co = \frac{1}{N} \sum_{k = 1}^{N} \lVert \vec{CoM} - \vec{p_i} \rVert
$$