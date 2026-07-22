#!/usr/bin/env bash
set -Eeuo pipefail

# Source the selected vehicle profile, then apply an explicit simulator-only
# hydrodynamic experiment overlay when requested.  With the flag false, the
# profile remains the sole owner of these physical parameters.
profile_file="$1"
override_enabled="$2"
base_mass_kg="$3"
fluid_density_kg_m3="$4"
linear_damping_x="$5"
quadratic_damping_x="$6"
shift 6

set -a
# shellcheck disable=SC1090
source "$profile_file"
set +a

if [[ "${override_enabled,,}" == "true" ]]; then
  export HERON_BASE_MASS="$base_mass_kg"
  export HERON_FLUID_DENSITY="$fluid_density_kg_m3"
  export HERON_LINEAR_DAMPING_X="$linear_damping_x"
  export HERON_QUADRATIC_DAMPING_X="$quadratic_damping_x"
fi

exec "$@"
