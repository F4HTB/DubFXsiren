#include <Arduino.h>
#include <driver/i2s_std.h>

// ------------------- Définition des broches et paramètres I2S -------------------
#define I2S_NUM I2S_NUM_0     // On utilise le canal I2S n°0
#define I2S_BCLK GPIO_NUM_26  // Broche du Bit Clock pour PCM5102
#define I2S_LRCK GPIO_NUM_25  // Broche du LRCK (Word Select) pour PCM5102
#define I2S_DOUT GPIO_NUM_22  // Broche de la Data out (DATA) vers PCM5102
#define SAMPLE_RATE 96000     // Taux d'échantillonnage (96 kHz)

// Handle du canal I2S (nouvelle API)
static i2s_chan_handle_t tx_chan;  // Contiendra l'handle du canal I2S (sortie audio)

// ----------------------- PINS / GLOBALS -----------------------------
#define LEDPIN 2  // LED sur la carte (ESP32), pour usage indicatif

// Potentiomètres reliés aux broches ADC de l’ESP32
const int POT_PIN[4] = { 36, 39, 34, 35 };
uint16_t POT_STATES[4] = { 0, 0, 0, 0 };
//  - POT_PIN[0] : Contrôle de la fréquence principale
//  - POT_PIN[1] : Contrôle de la fréquence de modulation
//  - POT_PIN[2] : Contrôle de la quantité d’avance dans le Delay
//  - POT_PIN[3] : Contrôle du Feedback (rétroaction) du Delay

// Boutons (pull-up interne, câblés au GND)
const int BUTTONS_PIN[6] = { 13, 12, 14, 27, 32, 33 };
uint8_t BUTTON_STATES[6] = { 0, 0, 0, 0, 0, 0 };
// Les états possibles sont :
//   0 = idle
//   1 = just released
//   2 = held
//   3 = just pressed
//
// Signification (par exemple) :
//   - BUTTONS_PIN[0] = Trigger
//   - BUTTONS_PIN[1] = Scale ou Drop
//   - BUTTONS_PIN[2] & [3] = Sélection forme d'onde modFreq
//   - BUTTONS_PIN[4] & [5] = Sélection forme d'onde mainFreq
// (Voir code pour leur usage précis)

// Attack-Sustain-Release
#define FADEMAX 512
static unsigned int FADE = 0;  // Variable pour gérer l’enveloppe d’amplitude (A-S-R)

// Buffer de délai
#define MAXDELAYBUFFERSIZE 40000
int16_t DELAYBUFFER[MAXDELAYBUFFERSIZE];   // Stocke les échantillons pour le Delay
static unsigned int delayWriteIndex = 0;   // Index principal d'écriture dans le buffer
static unsigned int pDelayWriteIndex = 0;  // Index précédent (pour interpoler ou remplir)

// ----------------------- Table de sinus en 16 bits ---------------------------
#define SINE_TABLE_SIZE 1024
static int16_t sine_table[SINE_TABLE_SIZE] = {
  // Table complète de 1024 points, -32768..32767, pour générer facilement un sinus
  // (Contient des valeurs positives et négatives, en "onde complète")
  0, 201, 402, 603, 804, 1005, 1206, 1407, 1608, 1809, 2009, 2210, 2410, 2611, 2811, 3012, 3212, 3412, 3612, 3811, 4011, 4210, 4410, 4609, 4808, 5007, 5205, 5404, 5602, 5800, 5998, 6195, 6393, 6590, 6786, 6983, 7179, 7375, 7571, 7767, 7962, 8157, 8351, 8545, 8739, 8933, 9126, 9319, 9512, 9704, 9896, 10087, 10278, 10469, 10659, 10849, 11039, 11228, 11417, 11605, 11793, 11980, 12167, 12353, 12539, 12725, 12910, 13094, 13279, 13462, 13645, 13828, 14010, 14191, 14372, 14553, 14732, 14912, 15090, 15269, 15446, 15623, 15800, 15976, 16151, 16325, 16499, 16673, 16846, 17018, 17189, 17360, 17530, 17700, 17869, 18037, 18204, 18371, 18537, 18703, 18868, 19032, 19195, 19357, 19519, 19680, 19841, 20000, 20159, 20317, 20475, 20631, 20787, 20942, 21096, 21250, 21403, 21554, 21705, 21856, 22005, 22154, 22301, 22448, 22594, 22739, 22884, 23027, 23170, 23311, 23452, 23592, 23731, 23870, 24007, 24143, 24279, 24413, 24547, 24680, 24811, 24942, 25072, 25201, 25329, 25456, 25582, 25708, 25832, 25955, 26077, 26198, 26319, 26438, 26556, 26674, 26790, 26905, 27019, 27133, 27245, 27356, 27466, 27575, 27683, 27790, 27896, 28001, 28105, 28208, 28310, 28411, 28510, 28609, 28706, 28803, 28898, 28992, 29085, 29177, 29268, 29358, 29447, 29534, 29621, 29706, 29791, 29874, 29956, 30037, 30117, 30195, 30273, 30349, 30424, 30498, 30571, 30643, 30714, 30783, 30852, 30919, 30985, 31050, 31113, 31176, 31237, 31297, 31356, 31414, 31470, 31526, 31580, 31633, 31685, 31736, 31785, 31833, 31880, 31926, 31971, 32014, 32057, 32098, 32137, 32176, 32213, 32250, 32285, 32318, 32351, 32382, 32412, 32441, 32469, 32495, 32521, 32545, 32567, 32589, 32609, 32628, 32646, 32663, 32678, 32692, 32705, 32717, 32728, 32737, 32745, 32752, 32757, 32761, 32765, 32766, 32767, 32766, 32765, 32761, 32757, 32752, 32745, 32737, 32728, 32717, 32705, 32692, 32678, 32663, 32646, 32628, 32609, 32589, 32567, 32545, 32521, 32495, 32469, 32441, 32412, 32382, 32351, 32318, 32285, 32250, 32213, 32176, 32137, 32098, 32057, 32014, 31971, 31926, 31880, 31833, 31785, 31736, 31685, 31633, 31580, 31526, 31470, 31414, 31356, 31297, 31237, 31176, 31113, 31050, 30985, 30919, 30852, 30783, 30714, 30643, 30571, 30498, 30424, 30349, 30273, 30195, 30117, 30037, 29956, 29874, 29791, 29706, 29621, 29534, 29447, 29358, 29268, 29177, 29085, 28992, 28898, 28803, 28706, 28609, 28510, 28411, 28310, 28208, 28105, 28001, 27896, 27790, 27683, 27575, 27466, 27356, 27245, 27133, 27019, 26905, 26790, 26674, 26556, 26438, 26319, 26198, 26077, 25955, 25832, 25708, 25582, 25456, 25329, 25201, 25072, 24942, 24811, 24680, 24547, 24413, 24279, 24143, 24007, 23870, 23731, 23592, 23452, 23311, 23170, 23027, 22884, 22739, 22594, 22448, 22301, 22154, 22005, 21856, 21705, 21554, 21403, 21250, 21096, 20942, 20787, 20631, 20475, 20317, 20159, 20000, 19841, 19680, 19519, 19357, 19195, 19032, 18868, 18703, 18537, 18371, 18204, 18037, 17869, 17700, 17530, 17360, 17189, 17018, 16846, 16673, 16499, 16325, 16151, 15976, 15800, 15623, 15446, 15269, 15090, 14912, 14732, 14553, 14372, 14191, 14010, 13828, 13645, 13462, 13279, 13094, 12910, 12725, 12539, 12353, 12167, 11980, 11793, 11605, 11417, 11228, 11039, 10849, 10659, 10469, 10278, 10087, 9896, 9704, 9512, 9319, 9126, 8933, 8739, 8545, 8351, 8157, 7962, 7767, 7571, 7375, 7179, 6983, 6786, 6590, 6393, 6195, 5998, 5800, 5602, 5404, 5205, 5007, 4808, 4609, 4410, 4210, 4011, 3811, 3612, 3412, 3212, 3012, 2811, 2611, 2410, 2210, 2009, 1809, 1608, 1407, 1206, 1005, 804, 603, 402, 201, 0, -201, -402, -603, -804, -1005, -1206, -1407, -1608, -1809, -2009, -2210, -2410, -2611, -2811, -3012, -3212, -3412, -3612, -3811, -4011, -4210, -4410, -4609, -4808, -5007, -5205, -5404, -5602, -5800, -5998, -6195, -6393, -6590, -6786, -6983, -7179, -7375, -7571, -7767, -7962, -8157, -8351, -8545, -8739, -8933, -9126, -9319, -9512, -9704, -9896, -10087, -10278, -10469, -10659, -10849, -11039, -11228, -11417, -11605, -11793, -11980, -12167, -12353, -12539, -12725, -12910, -13094, -13279, -13462, -13645, -13828, -14010, -14191, -14372, -14553, -14732, -14912, -15090, -15269, -15446, -15623, -15800, -15976, -16151, -16325, -16499, -16673, -16846, -17018, -17189, -17360, -17530, -17700, -17869, -18037, -18204, -18371, -18537, -18703, -18868, -19032, -19195, -19357, -19519, -19680, -19841, -20000, -20159, -20317, -20475, -20631, -20787, -20942, -21096, -21250, -21403, -21554, -21705, -21856, -22005, -22154, -22301, -22448, -22594, -22739, -22884, -23027, -23170, -23311, -23452, -23592, -23731, -23870, -24007, -24143, -24279, -24413, -24547, -24680, -24811, -24942, -25072, -25201, -25329, -25456, -25582, -25708, -25832, -25955, -26077, -26198, -26319, -26438, -26556, -26674, -26790, -26905, -27019, -27133, -27245, -27356, -27466, -27575, -27683, -27790, -27896, -28001, -28105, -28208, -28310, -28411, -28510, -28609, -28706, -28803, -28898, -28992, -29085, -29177, -29268, -29358, -29447, -29534, -29621, -29706, -29791, -29874, -29956, -30037, -30117, -30195, -30273, -30349, -30424, -30498, -30571, -30643, -30714, -30783, -30852, -30919, -30985, -31050, -31113, -31176, -31237, -31297, -31356, -31414, -31470, -31526, -31580, -31633, -31685, -31736, -31785, -31833, -31880, -31926, -31971, -32014, -32057, -32098, -32137, -32176, -32213, -32250, -32285, -32318, -32351, -32382, -32412, -32441, -32469, -32495, -32521, -32545, -32567, -32589, -32609, -32628, -32646, -32663, -32678, -32692, -32705, -32717, -32728, -32737, -32745, -32752, -32757, -32761, -32765, -32766, -32767, -32766, -32765, -32761, -32757, -32752, -32745, -32737, -32728, -32717, -32705, -32692, -32678, -32663, -32646, -32628, -32609, -32589, -32567, -32545, -32521, -32495, -32469, -32441, -32412, -32382, -32351, -32318, -32285, -32250, -32213, -32176, -32137, -32098, -32057, -32014, -31971, -31926, -31880, -31833, -31785, -31736, -31685, -31633, -31580, -31526, -31470, -31414, -31356, -31297, -31237, -31176, -31113, -31050, -30985, -30919, -30852, -30783, -30714, -30643, -30571, -30498, -30424, -30349, -30273, -30195, -30117, -30037, -29956, -29874, -29791, -29706, -29621, -29534, -29447, -29358, -29268, -29177, -29085, -28992, -28898, -28803, -28706, -28609, -28510, -28411, -28310, -28208, -28105, -28001, -27896, -27790, -27683, -27575, -27466, -27356, -27245, -27133, -27019, -26905, -26790, -26674, -26556, -26438, -26319, -26198, -26077, -25955, -25832, -25708, -25582, -25456, -25329, -25201, -25072, -24942, -24811, -24680, -24547, -24413, -24279, -24143, -24007, -23870, -23731, -23592, -23452, -23311, -23170, -23027, -22884, -22739, -22594, -22448, -22301, -22154, -22005, -21856, -21705, -21554, -21403, -21250, -21096, -20942, -20787, -20631, -20475, -20317, -20159, -20000, -19841, -19680, -19519, -19357, -19195, -19032, -18868, -18703, -18537, -18371, -18204, -18037, -17869, -17700, -17530, -17360, -17189, -17018, -16846, -16673, -16499, -16325, -16151, -15976, -15800, -15623, -15446, -15269, -15090, -14912, -14732, -14553, -14372, -14191, -14010, -13828, -13645, -13462, -13279, -13094, -12910, -12725, -12539, -12353, -12167, -11980, -11793, -11605, -11417, -11228, -11039, -10849, -10659, -10469, -10278, -10087, -9896, -9704, -9512, -9319, -9126, -8933, -8739, -8545, -8351, -8157, -7962, -7767, -7571, -7375, -7179, -6983, -6786, -6590, -6393, -6195, -5998, -5800, -5602, -5404, -5205, -5007, -4808, -4609, -4410, -4210, -4011, -3811, -3612, -3412, -3212, -3012, -2811, -2611, -2410, -2210, -2009, -1809, -1608, -1407, -1206, -1005, -804, -603, -402, -201
};

// ----------------------- Variables d'interface computeSample ---------------------------
// Les deux fréquences clés, en « volatile » car modifiées depuis la tâche Interface
volatile float MainFreqOsc = 0;  // Fréquence de l’oscillateur principal
volatile float modFreqOsc = 0;   // Fréquence de l’oscillateur de modulation

volatile int Delay_advance = 0;   // Valeur de l’avance du Delay (influencée par POT_STATES[2])
volatile int Delay_feedback = 0;  // Valeur du feedback du Delay (influencée par POT_STATES[3])

// -------------------- Calcul audio par échantillon -------------------

/**
 * @brief  Force la valeur dans la plage [-32768, 32767].
 */
int clamp16(int val) {
  if (val < -32768) return -32768;
  if (val > 32767) return 32767;
  return val;
}

/**
 * @brief  Multiplie inputSample par volFactor (en format Q16), puis clamp à int16.
 *         volFactor ~ 65536 = 1.0
 */
int volumeMult16(int inputSample, int volFactor) {
  int val = ((int32_t)inputSample * volFactor) >> 16;  // Décalage de 16 bits pour compenser
  return clamp16(val);
}

/**
 * @brief  Fait un mix linéaire entre sampleA et sampleB, mixFactor en Q16.
 *         mixFactor=32768 = 0.5 => 50/50
 */
int volumeMix16(int sampleA, int sampleB, int mixFactor) {
  int mixed = ((int32_t)sampleA * mixFactor + (int32_t)sampleB * (65536 - mixFactor)) >> 16;
  return clamp16(mixed);
}

/**
 * @brief  Additionne deux échantillons 16 bits avec clamp sur [-32768, 32767].
 */
int volumeAdd16(int sampleA, int sampleB) {
  return clamp16(sampleA + sampleB);
}

/**
 * @brief  Interpolation cubique dans une table de int16_t pour index et fraction donnés.
 *         Utilisée pour avoir un sinus plus « propre » (Catmull-Rom).
 */
int32_t cubicInterpolate(const int16_t* table, uint16_t size, uint16_t index, float frac) {
  // Indices nécessaires (on gère l'enroulement mod size)
  int idx0 = (index + size - 1) % size;
  int idx1 = index;
  int idx2 = (index + 1) % size;
  int idx3 = (index + 2) % size;

  // Conversion en float pour calcul
  float P0 = (float)table[idx0];
  float P1 = (float)table[idx1];
  float P2 = (float)table[idx2];
  float P3 = (float)table[idx3];

  // Formule Catmull-Rom
  float a = (-0.5f * P0) + (1.5f * P1) - (1.5f * P2) + (0.5f * P3);
  float b = (P0) - (2.5f * P1) + (2.0f * P2) - (0.5f * P3);
  float c = (-0.5f * P0) + (0.5f * P2);
  float d = P1;

  // Polynomial sur la fraction
  float result = ((a * frac + b) * frac + c) * frac + d;
  return (int32_t)result;
}

/**
 * @brief  Calcule un échantillon audio (mono) en tenant compte :
 *         - De l’oscillateur principal (formes d’onde : sinus, dent de scie, carré)
 *         - De l’oscillateur de modulation (change la phase ou amplitude)
 *         - De l’enveloppe ASR (FADE)
 *         - Du Delay avec feedback
 */
int16_t computeSample() {
  // Constantes et masques pour extraire la fraction de phase
  static const uint32_t PHASE_FRAC_MASK = (1UL << 22) - 1;  // Masque sur 22 bits (pour fraction)
  static const float TWO_TO_32 = 4294967296.0f;             // 2^32, référence pour la phase

  // Oscillateur principal
  static uint32_t phase = 0;         // Phase cumulée (32 bits, on avance par phase_inc)
  static float smoothedFreq = 0.0f;  // Fréquence lissée du main oscillator

  // Oscillateur de modulation
  static uint32_t modPhase = 0;         // Phase cumulée du modulateur
  static float smoothedModFreq = 0.0f;  // Fréquence lissée du modulateur

  int16_t Sample = 0;  // Échantillon final calculé

  // Réinitialise modPhase si un certain bouton est pressé (BUTTONS[1] == 3)
  if (BUTTON_STATES[1] == 3) {
    modPhase = 0;
  }

  // Si un des triggers est actif ou si l’enveloppe FADE est > 0, on calcule le son
  if ((BUTTON_STATES[0] == 2) || (BUTTON_STATES[1] == 2) || (FADE > 0)) {

    // --------------------------------------------------------------------
    // 1) OSCILLATEUR DE MODULATION (LFO ou modulateur de phase)
    // --------------------------------------------------------------------
    // On met à jour la fréquence de modulation avec un lissage exponentiel
    const float alphaMod = 0.008f;
    smoothedModFreq = alphaMod * modFreqOsc + (1.0f - alphaMod) * smoothedModFreq;

    // Conversion de la fréquence en incrément de phase
    const float modPhaseScale = TWO_TO_32 / SAMPLE_RATE;
    modPhase += (uint32_t)(smoothedModFreq * modPhaseScale);

    int16_t modSample = 0;

    // On va extraire la fraction de la phase (partie basse de modPhase)
    const float invPhaseScale = 1.0f / (float)(1UL << 22);

    // Choix de la forme d’onde du modulateur selon certains boutons
    if (BUTTON_STATES[3] == 2) {
      // Mode sinusoïdal avec interpolation linéaire
      uint16_t modIndex = ((modPhase >> 22) + 768) % SINE_TABLE_SIZE;
      uint32_t modFrac = modPhase & PHASE_FRAC_MASK;
      float fractionMod = modFrac * invPhaseScale;
      // On interpole linéairement entre sine_table[modIndex] et le point suivant
      int32_t interpolated = (int32_t)(((1.0f - fractionMod) * sine_table[modIndex])
                                       + (fractionMod * sine_table[(modIndex + 1) % SINE_TABLE_SIZE]));
      // Décalage de l’amplitude finale (de [-32768,32767] vers [0,16384])
      //   +32767 => ramène dans [0,65535], puis /4 => [0,16384]
      modSample = (int16_t)((interpolated + 32767) / 4);

    } else if (BUTTON_STATES[2] == 2) {
      // Mode dent de scie : partie haute de modPhase, divisée par 4
      modSample = (int16_t)((modPhase >> 16) / 4);

    } else {
      // Mode carré : 0 ou 16384 selon le bit le plus haut de modPhase
      modSample = (modPhase & 0x80000000) ? 16384 : 0;
    }

    // --------------------------------------------------------------------
    // 2) OSCILLATEUR PRINCIPAL
    // --------------------------------------------------------------------
    // Lissage exponentiel pour la fréquence principale
    const float alpha = 0.008f;
    smoothedFreq = alpha * MainFreqOsc + (1.0f - alpha) * smoothedFreq;

    // Conversion en incrément de phase (similaire au modPhaseScale)
    const float phaseIncScale = TWO_TO_32 / SAMPLE_RATE;
    uint32_t phase_inc = (uint32_t)(smoothedFreq * phaseIncScale);

    // On applique la modulation de phase, avec une « profondeur » modDepth
    const float modDepth = 512.0f;
    int32_t scaledMod = (int32_t)(modSample * modDepth);
    phase += phase_inc + scaledMod;

    // Choix de la forme d'onde principale selon d’autres boutons
    if (BUTTON_STATES[4] == 2) {
      // Sinus avec interpolation cubique (plus propre qu’une simple interpolation linéaire)
      uint16_t index = phase >> 22;             // index dans la table
      uint32_t frac = phase & PHASE_FRAC_MASK;  // partie fractionnelle
      float fraction = (float)frac / (float)(1UL << 22);
      // Interpolation cubique à partir de la table de sinus
      Sample = (int16_t)cubicInterpolate(sine_table, SINE_TABLE_SIZE, index, fraction)*0.9f;

    } else if (BUTTON_STATES[5] == 2) {
      // Dent de scie
      Sample = (int16_t)((phase >> 16) - 32768)*0.75f;  // pour volume audio egale

    } else {
      // Carré
      Sample = (phase & 0x80000000) ? 26542 : -26542;  //  pour volume audio egale
    }

    // --------------------------------------------------------------------
    // 3) ASR ENVELOPE (FADE)
    // --------------------------------------------------------------------
    // FADE monte si on vient d’appuyer sur un bouton (BUTTON_STATES[0] ou [1])
    if ((FADE < FADEMAX) && ((BUTTON_STATES[0] == 2) || (BUTTON_STATES[1] == 2))) {
      FADE++;
    } else {
      // Sinon on redescend progressivement
      if (FADE > 0) FADE--;
    }
    // Application de l’enveloppe sur le Sample
    //  FADE << 7 donne un factor 0..65535 (1.0 => 65536)
    Sample = volumeMult16(Sample, FADE << 7);

  } else {
    // Aucune note jouée, ou plus de sustain => silence
    Sample = 0;
  }

  // --------------------------------------------------------------------
  // 4) DELAY & FEEDBACK
  // --------------------------------------------------------------------
  // Cette partie s’applique qu’on ait un son ou non ;
  // on écrit toujours dans le buffer de Delay.
  pDelayWriteIndex = delayWriteIndex;
  // On avance l’index d’écriture dans le buffer, en fonction de Delay_advance
  delayWriteIndex += (Delay_advance << 9);

  // On lit un ancien sample (oldsample) dans le buffer (pour feedback),
  // mix 50/50 entre deux positions
  int oldsample = volumeMix16(
    DELAYBUFFER[(delayWriteIndex >> 16) % MAXDELAYBUFFERSIZE],
    DELAYBUFFER[(pDelayWriteIndex >> 16) % MAXDELAYBUFFERSIZE],
    32768  // mixFactor = 0.5 en Q16
  );

  // Contrôle de saturation interne
  static int ringMin = 32767;
  static int ringMax = -32768;
  static unsigned long ringCount = 0;
  static const unsigned long DELAY_SIZE = MAXDELAYBUFFERSIZE;
  static int maxPossibleFbQ16 = 0;  // feedback max toléré

  // Si le feedback calculé dépasse la limite, on le ramène
  if (Delay_feedback > maxPossibleFbQ16) {
    Delay_feedback = maxPossibleFbQ16;
  }

  // On multiplie oldsample par le feedback
  oldsample = volumeMult16(oldsample, Delay_feedback);

  // On additionne le signal direct + feedback
  int SampleOut = volumeAdd16(oldsample, Sample);

  // Écriture dans le buffer de Delay (en cas de saut >1)
  for (unsigned int i = (pDelayWriteIndex >> 16); i < (delayWriteIndex >> 16); i++) {
    int idx = i % MAXDELAYBUFFERSIZE;

    int16_t val = clamp16(SampleOut);

    // Mise à jour de ringMin/ringMax pour surveiller l’amplitude
    if (val < ringMin) ringMin = val;
    if (val > ringMax) ringMax = val;

    // Écriture dans le buffer de Delay
    DELAYBUFFER[idx] = val;

    // Incrémente le compteur
    ringCount++;

    // Si on a écrit DELAY_SIZE échantillons, on réinitialise
    if (ringCount >= DELAY_SIZE) {
      // Pic absolu (positif ou négatif)
      int absMin = abs(ringMin);
      int absMax = abs(ringMax);
      int peak = (absMin > absMax) ? absMin : absMax;

      // On remet à zéro
      ringCount = 0;
      ringMin = 32767;
      ringMax = -32768;

      // On calcule la marge pour éviter la saturation
      if (peak > 0) {
        float margin = 0.95f;
        float maxPossibleFb = (32767.0f * margin) / (float)peak;
        maxPossibleFbQ16 = (int)(maxPossibleFb * 65536.0f);

        // Si l’actual feedback est > maxPossible, on le rabaisse
        if (Delay_feedback > maxPossibleFbQ16) {
          Delay_feedback = maxPossibleFbQ16;
        }
      }
    }
  }

  // On ramène l’index delayWriteIndex dans la plage valide
  delayWriteIndex %= (MAXDELAYBUFFERSIZE << 16);

  return SampleOut;  // Renvoie l’échantillon final (mono)
}

// ---------------------- SETUP I2S (nouvelle API) -------------------
void setupI2S() {
  // Configuration du canal I2S en mode maître
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM, I2S_ROLE_MASTER);
  i2s_new_channel(&chan_cfg, &tx_chan, NULL);

  // Configuration en standard I2S (16 bits, stéréo, protocole Philips)
  i2s_std_config_t std_cfg = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,  // PCM5102 n’a pas besoin du MCLK (peut marcher sans)
      .bclk = I2S_BCLK,
      .ws = I2S_LRCK,
      .dout = I2S_DOUT,
      .din = I2S_GPIO_UNUSED,
    },
  };

  // Initialisation du canal en mode standard, puis activation
  i2s_channel_init_std_mode(tx_chan, &std_cfg);
  i2s_channel_enable(tx_chan);
}

// ---------------------- SETUP & LOOP --------------------------------
void setup() {
  // Pas forcément besoin de Serial dans cette application,
  // mais si on veut déboguer on peut décommenter :
  // Serial.begin(115200);

  // Initialisation I2S
  setupI2S();

  // LED pin
  pinMode(LEDPIN, OUTPUT);

  // Boutons en pull-up
  for (int i = 0; i < 6; i++) {
    pinMode(BUTTONS_PIN[i], INPUT_PULLUP);
  }

  // Ajuste l’atténuation ADC (selon la tension max attendue)
  analogSetPinAttenuation(POT_PIN[0], ADC_11db);

  // Création des tâches FreeRTOS pour l’audio et l’interface
  xTaskCreatePinnedToCore(TaskAudio, "TaskAudio", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskInterface, "TaskInterface", 4096, NULL, 1, NULL, 1);
}

void loop() {
  // Vide, car tout se passe dans les tâches
}

// ---------------------- Tâche pour la génération audio --------------------------------
void TaskAudio(void* pvParameters) {
  const int NUM_SAMPLES = 128;                           // Nombre d’échantillons par bloc
  static int16_t sampleBuffer[NUM_SAMPLES * 2] = { 0 };  // Buffer stéréo (gauche/droite)

  while (1) {
    // Génère NUM_SAMPLES échantillons
    for (int i = 0; i < NUM_SAMPLES; i++) {
      int16_t audioOut = computeSample();  // On récupère l’échantillon mono
      // Duplique sur les deux canaux (L, R) pour la sortie stéréo
      sampleBuffer[2 * i + 0] = audioOut;
      sampleBuffer[2 * i + 1] = audioOut;
    }
    // Écriture dans le canal I2S
    size_t bytesWritten = 0;
    i2s_channel_write(tx_chan, sampleBuffer, sizeof(sampleBuffer), &bytesWritten, portMAX_DELAY);
  }
}

// ----------------------- Update de l'état des boutons et Potentiomètres -----------------------------

/**
 * @brief  Mise à jour de l’état des 6 boutons, avec une simple machine à états.
 */
void buttonUpdate() {
  for (int i = 0; i < 6; i++) {
    bool curr = (digitalRead(BUTTONS_PIN[i]) == LOW);  // Bouton appuyé = LOW
    // De "just pressed" (3) à "held" (2)
    if (BUTTON_STATES[i] == 3) BUTTON_STATES[i] = 2;
    // Si on détecte un appui depuis l’état idle => "just pressed" (3)
    if (curr && BUTTON_STATES[i] == 0) BUTTON_STATES[i] = 3;
    // "just released" (1) => repasse à idle (0)
    if (BUTTON_STATES[i] == 1) BUTTON_STATES[i] = 0;
    // Si le bouton n’est plus pressé et qu’on était en "held" => "just released"
    if (!curr && BUTTON_STATES[i] == 2) BUTTON_STATES[i] = 1;
  }
}

#define SAMPLE_WINDOW 10  // Durée de la fenêtre en ms
#define MAX_SAMPLES 20    // Nombre d'échantillons stockés pour le filtrage
#define ADC_MAX 4096      // Résolution ADC 12 bits sur ESP32
#define NUM_POTS 4

// Tampons circulaires pour lisser les lectures de chaque potentiomètre
static uint16_t values[NUM_POTS][MAX_SAMPLES] = { 0 };
static int bufferIndex[NUM_POTS] = { 0 };
static unsigned long lastUpdate = 0;

/**
 * @brief  Met à jour la lecture des potentiomètres en effectuant un filtrage médian.
 */
void potentiometerUpdate() {
  // On fait des lectures plus ou moins espacées
  if (millis() - lastUpdate < (SAMPLE_WINDOW / MAX_SAMPLES)) return;
  lastUpdate = millis();

  for (int pot = 0; pot < NUM_POTS; pot++) {
    // Lecture ADC brute
    uint16_t newReading = analogRead(POT_PIN[pot]);
    // Mise à jour du buffer circulaire
    values[pot][bufferIndex[pot]] = newReading;
    bufferIndex[pot] = (bufferIndex[pot] + 1) % MAX_SAMPLES;

    // On recopie dans un tableau intermédiaire pour le tri
    uint16_t sorted[MAX_SAMPLES];
    memcpy(sorted, values[pot], sizeof(sorted));

    // Tri par insertion pour avoir la médiane
    for (int i = 1; i < MAX_SAMPLES; i++) {
      uint16_t key = sorted[i];
      int j = i - 1;
      while (j >= 0 && sorted[j] > key) {
        sorted[j + 1] = sorted[j];
        j--;
      }
      sorted[j + 1] = key;
    }

    // On récupère la médiane (au centre)
    POT_STATES[pot] = sorted[MAX_SAMPLES / 2];
  }
}

// ---------------------- Tâche pour la gestion de l'interface utilisateur --------------------------------
void TaskInterface(void* pvParameters) {
  while (1) {
    // Lire et filtrer les potentiomètres
    potentiometerUpdate();
    // Lire l’état des boutons
    buttonUpdate();

    // Fréquence principale (POT[0]) : on mappe vers [25..3519] de manière exponentielle
    float adc_norm_freq = POT_STATES[0] / 4095.0f;
    float expFactor_freq = (exp(adc_norm_freq * 4.0f) - 1) / (exp(4.0f) - 1);
    MainFreqOsc = 25 + (3519 - 25) * expFactor_freq;

    // Fréquence de modulation (POT[1]) : on mappe [1..60] (en gros un LFO)
    float adc_norm_mod = POT_STATES[1] / 4095.0f;
    float expFactorMod = (exp(adc_norm_mod * 4.0f) - 1) / (exp(4.0f) - 1);
    modFreqOsc = (60.0f - 1.0f) * expFactorMod;

    // Contrôle de l'avancement du Delay (POT[2])
    float adc_norm_delay = POT_STATES[2] / 4095.0f;
    float expFactorDelay = (exp(adc_norm_delay * 4.0f) - 1) / (exp(4.0f) - 1);
    Delay_advance = (int)((1023UL) * expFactorDelay);

    // Contrôle du feedback (POT[3])
    // Simplifié ici :
    //Delay_feedback = ((((POT_STATES[3] / 4) * 3) >> 1) << 6);
    // => lecture >> 2, *3, >>1, <<6 : un certain scaling
    //   (vous pourriez ajuster la formule selon vos besoins)
    /*
    int v = ((((POT_STATES[3] / 4) * 3) >> 1) << 6), m = 65536, d = m / 10, l = m - d, h = m + d;
    if (v >= l && v <= h) v = m;
    Delay_feedback = v;
    */
    const int m=65536,d=m/10,l=m-d,h=m+d;
    int v=POT_STATES[3]*24;
    v=(v>l&&v<h)?m+int((v<m?-1:1)*pow(abs(float(v-m)/d),4.f)*d):(v>=l&&v<=h?m:v);Delay_feedback=v;

    // Allume la LED si le delayWriteIndex est en-deçà d’un seuil
    // ou si Delay_advance = 0 => LED = HIGH
    if ((delayWriteIndex < (20500 << 16)) || (Delay_advance == 0)) {
      digitalWrite(LEDPIN, HIGH);
    } else {
      digitalWrite(LEDPIN, LOW);
    }

    // Petit délai pour laisser respirer le CPU
    vTaskDelay(pdMS_TO_TICKS(6));
  }
}
