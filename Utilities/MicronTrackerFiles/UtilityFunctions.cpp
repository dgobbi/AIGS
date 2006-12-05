/**************************************************************
*
*     Micron Tracker: Example C++ wrapper and Multi-platform demo
*   
*     Written by: 
*			Shahram Izadyar, Robarts Research Institute - London- Ontario , www.robarts.ca
*			Claudio Gatti, Claron Technology - Toronto -Ontario, www.clarontech.com
*
*     Copyright Claron Technology 2000-2003
*
***************************************************************/

#ifndef WIN32
#include <unistd.h>
#endif
#include "UtilityFunctions.h"


/****************************/
/** Copies the current path in the passed argument (currPath). */
void mtUtils::getCurrPath(char* currPath)
{
	int currDirResult = 1;
	const short buffer = 255;
	char currDir[255];

	if( getcwd(currDir, buffer) == NULL )
		strcpy(currPath, "ERROR");
	else
		strcpy(currPath,currDir);
}

/****************************/
// This function assumes the passed dir argument (the passed directory) resides on the current directory.
// It just finds the file names, not the subdirectories
int mtUtils::getFileNamesFromDirectory(vector<string> &fileNames, char* dir, bool returnCompletePath)
{
	#if(WIN32)
	{
		_finddata_t file; 
		char currentFolderPath[255];
		mtUtils::getCurrPath(currentFolderPath);
		//if NULL is passed as the last argument, search the current directory
		if ( dir != NULL )
		{
			strcat(currentFolderPath, "\\");
			strcat(currentFolderPath, dir);
			strcat(currentFolderPath, "\\");
		}

		//change the directory to the currrent\Markers (as done above)
		if (_chdir(currentFolderPath) == 0) 
		{
			long currentPosition = _findfirst("*.*", &file); //find the first file in directory
			if (currentPosition == -1L)
				return 0; //end the procedure if no file is found
			else
			{
				do
				{
					string fileName = file.name;
					//ignore . and ..
					if(strcmp(fileName.c_str() , ".") != 0 && strcmp(fileName.c_str() , "..") !=0 )
					{
						//If not subdirectory
						if (!(file.attrib & _A_SUBDIR))
						{
							//If returnCompletePath is true return the full path of the files, otherwise return just the file names.
							returnCompletePath ? fileNames.push_back(currentFolderPath +  fileName) : fileNames.push_back(fileName) ;
						}
					}
				}while(_findnext(currentPosition, &file) == 0);
			}
			_findclose(currentPosition); //close search
			// Return to the parent directory (get out of the Markers folder) ******Critical******
			_chdir("..");
			return 1;
		}
		return 0;
	}
	#else
	{
		DIR* d;
		struct dirent *ent;

		char currentFolderPath[255];
		mti_utils::getCurrPath(currentFolderPath);
		//if NULL is passed as the last argument, search the current directory
		if ( dir != NULL )
		{
			strcat(currentFolderPath, "/");
			strcat(currentFolderPath, dir);
			strcat(currentFolderPath, "/");
		}
		if ( (d = opendir(currentFolderPath)) != NULL)
		{
			while ((ent = readdir(d)) != NULL)
			{
				string entry( ent->d_name );
				if (entry != "." && entry != "..")
					//If returnCompletePath is true, return the full path of the files, otherwise return just the file names.
				    returnCompletePath ? fileNames.push_back(currentFolderPath + entry) : fileNames.push_back(entry);
			}
		}
		closedir(d);
		return 0;

	}
	#endif
}


/****************************/
/** This function renames file 'oldName' to the 'newName'. Both files reside on the 'dir' folder
in the current path. Returns 0 if successful, -1 if not. */
int mtUtils::renameFile(string oldName, string newName, char* dir)
{
	std::cout << "old name is " << oldName << " new name " << newName << std::endl;
	char currentFolderPath[255];
	mtUtils::getCurrPath(currentFolderPath);
	if ( dir != NULL )
	{
		#if(WIN32)
		{
			strcat(currentFolderPath, "\\");
			strcat(currentFolderPath, dir);
			strcat(currentFolderPath, "\\");
		}
		#else
		{
			strcat(currentFolderPath, "/");
			strcat(currentFolderPath, dir);
			strcat(currentFolderPath, "/");
		}
		#endif
	}

	if (_chdir(currentFolderPath) == 0 )
	{
		int result = rename(oldName.c_str(), newName.c_str());
		// Return to the parent directory
		_chdir("..");
		if (result == 0 )
			return 0; // successful
		else // not successful
			return -1;
	}
	return -1;
}


/****************************/
/** This function removes file 'fileName' from the 'dir' folder in the current path.
Returns 0 if successful, -1 if not. */
int mtUtils::removeFile(string fileName, char* dir)
{
	char currentFolderPath[255];
	mtUtils::getCurrPath(currentFolderPath);
	if ( dir != NULL )
	{
		#if(WIN32)
		{
			strcat(currentFolderPath, "\\");
			strcat(currentFolderPath, dir);
			strcat(currentFolderPath, "\\");
		}
		#else
		{
			strcat(currentFolderPath, "/");
			strcat(currentFolderPath, dir);
			strcat(currentFolderPath, "/");
		}
		#endif
	}
	if (_chdir(currentFolderPath) == 0)
	{
		int result = remove(fileName.c_str());
		// Return to the parent directory
		_chdir("..");
		if (result == 0)
			return 0; // successful
		else // not successful
			return -1;
	}
	return -1;
}

double mtUtils::acos(double x)
{
	if (x==1)
		return 0;
	else if (x==-1)
		return PI / 2.0;
	else 
		return atan((double) ( -x / sqrt(-x * x + 1)) + 2 * atan(1.0));
}

string mtUtils::handleErrors(int errorNum)
{
	/*mtOK=0,
	mtError,
	mtInvalidHandle,
	mtNullPointer,
	mtOutOfMemory,
	mtStringParamTooLong,
	mtPathNotSet,
	mtWriteToFileFailed,
	mtInvalidIndex,
	mtInvalidSideI,
	mtInvalidDivisor,
	mtEmptyCollection,
	mtInsufficientSamples,
	mtOddNumberOfSamples,
	mtLessThan2Vectors,
	mtMoreThanMaxVectorsPerFacet,
	mtErrorExceedsTolerance,
	mtInsufficientAngleBetweenVectors,
	mtFirstVectorShorterThanSecond,
	mtVectorLengthsTooSimilar,
	mtNullTemplateVector,
	mtDifferentFacetsGeometryTooSimilar,
	mtNoncompliantFacetDefinition,
	mtCollectionContainsNonVectorHandles,
	mtParameterOutOfRange,
	mtEmptyBuffer,
	mtPropertyNotSet,
	mtDimensionsDoNotMatch,
	mtOpenFileFailed,
	mtReadFileFailed,
	mtNotACalibrationFile,
	mtIncorrectFileVersion,
	mtCalibrationFileIncomplete,
	mtCameraInitializeFailed,
	mtGrabFrameError*/
	string error = "";
	switch (errorNum)
	{
	case 1:
		error = "Unknown error!";
		break;
	case 2:
		error = "Invalid handle!";
		break;
	case 3:
		error = "Null pointer!";
		break;
	case 4:
		error = "Out of memory";
	case 5:
		error = "String parameter is too long!";
		break;
	default:
		error = "Unknown error";
		break;
	}

	return error;
}