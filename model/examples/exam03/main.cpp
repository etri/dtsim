/**
 * Repast-City on RepastHPC
 */

#include <unistd.h>
#include <getopt.h>
#include <curl/curl.h>

#include "city/DTSim.h"

static char *prog = NULL;

static void usage(void)
{
	printf("Usage: %s [-m model_property_file]\n", prog);
}

static void do_exam03()
{
	printf("\n");

	/*********************************************************/
	/* check if specific parameter exists in the model.props */
	/*********************************************************/
	if (dtsim::City::instance()->containProperty("buildings.file.path")) {
		printf("'buildings.file.path' exists in model.props\n");
	} else {
		printf("'buildings.file.path' does not exist in model.props\n");
	}

	if (dtsim::City::instance()->containProperty("dummy.param")) {
		printf("'dummy.param' exists in model.props\n");
	} else {
		printf("'dummy.param' does not exist in model.props\n");
	}

	/*********************************************************/
	/* get INT parameter from the model.props                */
	/*********************************************************/
	int idField = dtsim::City::instance()->getIntProperty("bus.lines.id.field");
	printf("INT('bus.lines.id.field') value -> %d\n", idField);

	/*********************************************************/
	/* get STRING parameter from the model.props             */
	/*********************************************************/
	string stateDir = dtsim::City::instance()->getStringProperty("people.state.dir");
	printf("STRING('people.state.dir') value -> %s\n", stateDir.c_str());

	/*********************************************************/
	/* get User-Defined parameter from the model.props       */
	/*********************************************************/
	int udInt = dtsim::City::instance()->getIntProperty("user.defined.int");
	string udString = dtsim::City::instance()->getStringProperty("user.defined.string");
	printf("'user.defined.int' -> %d\n", udInt);
	printf("'user.defined.string' -> %s\n", udString.c_str());
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
		printf("=============================\n");
		printf("[exam03] get model parameters\n");
		printf("=============================\n");
		printf("Building Virtual City....\n\n");

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
	do_exam03();

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
