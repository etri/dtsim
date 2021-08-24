/**
 * Repast-City on RepastHPC
 */

#include <unistd.h>
#include <getopt.h>
#include <curl/curl.h>

#include "city/DTSim.h"
#include "Oracle.h"

static char *prog = NULL;

static void usage(void)
{
	printf("Usage: %s [-m model_property_file]\n", prog);
}

static void do_exam02()
{
	Oracle *city = new Oracle();

	city->cityLookup();

	delete city;
}

int main(int argc, char **argv)
{
	string modelPropsFile;
	int ret = 0;
	int iteration = 0;
	int c;

	prog = argv[0];

	static const struct option options[] = {
		{"help", no_argument, 0, 'h'},
		{0,      0,           0,  0}
	};

	while ((c = getopt_long(argc, argv, "hm:v", options, NULL)) != -1) {
		switch (c) {
		case 'm':
			if (!optarg) {
				usage();
				return -1;
			}
			modelPropsFile = optarg;
			break;

		case 'h':
		default:
			usage();
			return -1;
		}
	}

	/*********************************************/
	/* STEP1 : Initialize City Environment       */
	/*********************************************/
	try { 
		printf("=======================================\n");
		printf("[exam02] lookup City environment agents\n");
		printf("=======================================\n");
		printf("Building Virtual City....\n");

		dtsim::City::init(modelPropsFile);
		dtsim::City::instance()->buildEnvironment();
	} catch (dtsim::CityException& e) {
		cout << e.what() << endl;
		ret = -1;
		goto out;
	}

	/*********************************************/
	/* STEP2 : Run Model Functions               */
	/*********************************************/
	do_exam02();

	/*********************************************/
	/* STEP3 : Destroy City Environment          */
	/*********************************************/
	try {
		printf("\nFinalized Virtual City....\n");
		dtsim::CityScheduler* scheduler = dtsim::City::instance()->getScheduler();
		dtsim::City::exit();
	} catch(dtsim::CityException& e) {
		cout << e.what() << endl;
		ret = -1;
	}

out:

	return ret;
}
