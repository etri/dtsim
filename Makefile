#
# Makefil
#
# $Revision$
# $LastChangedDate$
#

all: repast_hpc city 

repast_hpc:
	@ $(MAKE) -C src/repast_hpc install

city:
	@ $(MAKE) -C src/city install

examples:
	@ $(MAKE) -C model/examples 

clean:
	@ $(MAKE) -C src/repast_hpc clean
	@ $(MAKE) -C src/city clean
	@ $(MAKE) -C model/examples clean

clear:
	@ $(MAKE) -C src/repast_hpc clear
	@ $(MAKE) -C src/city clear
	@ $(MAKE) -C model/examples clear

clean-all:
	@ $(MAKE) -C src/repast_hpc clean-all
	@ $(MAKE) -C src/city clean-all
	@ $(MAKE) -C model/examples clean-all
