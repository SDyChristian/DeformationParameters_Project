/* fopen example */
#include <stdio.h>
#include <stdlib.h>

int main(void)
{
	double SomeVariables[3] = {0,1,2}; // self explainatory
	char *Data = (char *)malloc(200); // array to be used for storing data

	// write data in string formatted way
	sprintf(Data,"%lf %lf %lf\n",SomeVariables[0],SomeVariables[1],SomeVariables[2]);

	FILE * pFile = fopen("SavedData.txt","wa"); // will create the file (if not existent) and append consecutive data writing 
	if (pFile != NULL)
	{
		fputs(Data, pFile);
		fclose (pFile);
	}

	free(Data);
	return 0;
}



