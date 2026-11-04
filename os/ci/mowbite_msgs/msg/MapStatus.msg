# Latched on map/status: whether the map fits the datum of tron_gnss. Mowing needs DATUM_MATCH.
uint8 DATUM_UNKNOWN=0
uint8 DATUM_EMPTY=1
uint8 DATUM_MATCH=2
uint8 DATUM_MISMATCH=3
uint8 DATUM_UNBOUND=4

# DATUM_UNKNOWN: tron_gnss has not told its datum yet
# DATUM_EMPTY: no areas and no docking station, nothing to tie to a datum
# DATUM_MATCH: the map was recorded in the datum in use
# DATUM_MISMATCH: the map was recorded in another datum
# DATUM_UNBOUND: the map is there, but not the datum it was recorded in
uint8 datum_state
bool mowing_allowed
# Why mowing is not allowed, empty if it is
string reason
uint32 areas
uint32 docking_stations
