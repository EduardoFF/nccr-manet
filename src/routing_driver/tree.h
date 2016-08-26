
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <map>
#include <vector>
typedef struct EntryData EntrydData;
typedef struct Tree Tree;

using namespace std;

struct EntryData
{
	string finalDest;
	string destNode;
	int weight;
};

struct Tree
{
	int timestamp;
	int nbNode;
    map<string, vector<EntryData> > rTable;
};

/*int main()
{
	EntryData entry;
	Tree tree;
	vector <EntryData> entry;

	entry.push_back(EntryData());
	entry[0].finalDest = "CC";
	entry[0].destNode = "0";
	entry[0].weigth = 0;
	for(int i=0;i< entry.size();i++)
	{
		cout << entry[i] << endl;
	}


	std::map<std::string, vector<EntryData>> rTable;
	for(int i=0; i<nbNode;i++)
	{
		char nbNode[10];
		sprintf(nbNode, "%d",(i+1));
		tree.finalDest = "CC";
		tree.destNode = nbNode;
		entry.push_back(EntryData());
		for (int j=0;j<entry.size();j++)
		{
			char destNode[10];
	    		sprintf(destNode, "%d",j);
			entry[j].finalDest = "CC";
			entry[j].destNode = destNode;
			entry[j].weigth = j;
		}
		cout << "The routing table for Node " << i << "is " << rTable["nbNode"] << endl;
	}
}
*/

