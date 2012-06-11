#include "SIR.h"

template<class Particles>
SIR<Particles>::SIR(int nbParticles, Particles& model) : _Filter<Particles>(nbParticles, model)
{
}
