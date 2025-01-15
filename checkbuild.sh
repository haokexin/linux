#/bin/bash

RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' 

./make_c1200_db.sh || { echo -e "${RED}Build error on ./make_c1200_db.sh${NC}" ; exit 1; }
./make_c1200_adas.sh || { echo  -e "${RED}Build error on ./make_c1200_adas.sh${NC}" ; exit 1; }
./make_c1200_adas_slt.sh || { echo -e "Build error on ./make_c1200_adas_slt.sh" ; exit 1; }
./make_c1200_adas_mass.sh || { echo -e "${RED}Build  error on ./make_c1200_adas_mass.sh${NC}" ; exit 1; }
./make_c1200_db_slt.sh || { echo -e "${RED}Build  error on ./make_c1200_db_slt.sh${NC}" ; exit 1; }
./make_c1200_db_mass.sh || { echo -e "${RED}Build  error on ./make_c1200_db_mass.sh${NC}" ; exit 1; }
./make_c1200_ivi.sh || { echo -e "${RED}Build  error on ./make_c1200_ivi.sh${NC}"; exit 1; }
./make_c1200_recovery.sh || { echo -e "${RED}Build  error on ./make_c1200_recovery.sh${NC}" ; exit 1; }

echo -e "${YELLOW}Build checking PASS!!!${NC}"
