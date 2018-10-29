/**
 * File: Demo.cpp
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: demo application of DBoW2
 * License: see the LICENSE.txt file
 */

#include <iostream>
#include <vector>
#include <io.h>

// DBoW2
#include <DBoW2/DBoW2.h> // defines OrbVocabulary and OrbDatabase

#include <DUtils/DUtils.h>
#include <DVision/DVision.h>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <map>

using namespace DBoW2;
using namespace DUtils;
using namespace std;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

int loadVideoFeatures(const char *videofilename, vector<vector<cv::Mat > > &features);
void changeStructure(const cv::Mat &plain, vector<cv::Mat> &out);
void testVocCreation(const vector<vector<cv::Mat > > &features, std::map<unsigned int, unsigned int> &vpair);
void testDatabase(const vector<vector<cv::Mat > > &features);



// ----------------------------------------------------------------------------
int readtimestamp(const char *timestampfilename, std::map<unsigned int, unsigned long long> &midx)
{
	FILE *fp = fopen(timestampfilename, "r");
	if (!fp)
		return -2;
	unsigned int count = 0;
	unsigned long long timestamp;
	while (!feof(fp))
	{
		unsigned long long sec, usec;
		fscanf(fp, "%lld %lld\n", &sec, &usec);
		timestamp = sec * 1000000ULL + usec;
		std::map<unsigned int, unsigned long long>::iterator iter = midx.find(count);
		if (iter != midx.end())
		{
			iter->second = timestamp;
		}
		count++;
	}
	return 0;
}

int loopclosedetect(const char *videofilename,
	const char *timestampfilename,
	std::vector<std::pair<unsigned long long, unsigned long long>>& vclosedpair)
{
	std::map<unsigned int, unsigned int> vpair;
	vector<vector<cv::Mat > > features;
	loadVideoFeatures(videofilename, features);

	testVocCreation(features, vpair);

	std::map<unsigned int, unsigned long long> midx;
	std::map<unsigned int, unsigned int>::iterator iter = vpair.begin();
	for(;iter != vpair.end();++iter)
	{
		int id1 = iter->first;
		int id2 = iter->second;
		midx[id1] = 0;
		midx[id2] = 0;
	}

	readtimestamp(timestampfilename, midx);

	vclosedpair.clear();
	iter = vpair.begin();
	for (; iter != vpair.end(); ++iter)
	{
		int id1 = iter->first;
		int id2 = iter->second;
		
		vclosedpair.push_back(std::pair<unsigned long long, unsigned long long>(midx[id1], midx[id2]));
	}
	return 0;
}

int loadVideoFeatures(const char *videofilename, vector<vector<cv::Mat > > &features)
{
	cv::VideoCapture capture;
	if (!capture.open(videofilename))
	{
		return -1;
	}

	int frame_num = 0;
	//char filename[255];
	if (capture.isOpened())
	{
		int frame_num = capture.get(cv::CAP_PROP_FRAME_COUNT);
		int NIMAGES = frame_num/10;
		std::cout << NIMAGES << endl;

		features.clear();
		features.reserve(NIMAGES);
		cv::Mat image;

		cv::Ptr<cv::ORB> orb = cv::ORB::create();
		for(int i = 0;i < frame_num;i++)
		{
			cv::Mat mask;
			vector<cv::KeyPoint> keypoints;
			cv::Mat descriptors;

			capture >> image;

			if (i % 10 == 0)
			{
				orb->detectAndCompute(image, mask, keypoints, descriptors);

				features.push_back(vector<cv::Mat >());
				changeStructure(descriptors, features.back());
				std::cout << "Extracting ORB features " << i << endl;

			}
		}
	}
	capture.release();
	

	std::cout << "Extracting ORB features..." << endl;
	return 1;
}

// ----------------------------------------------------------------------------

void changeStructure(const cv::Mat &plain, vector<cv::Mat> &out)
{
  out.resize(plain.rows);

  for(int i = 0; i < plain.rows; ++i)
  {
    out[i] = plain.row(i);
  }
}

// ----------------------------------------------------------------------------

void testVocCreation(const vector<vector<cv::Mat > > &features, std::map<unsigned int, unsigned int> &vpair)
{
  // branching factor and depth levels 
  const int k = 9;
  const int L = 3;
  const WeightingType weight = TF_IDF;
  const ScoringType score = L1_NORM;

  OrbVocabulary voc(k, L, weight, score);

  std::cout << "Creating a small " << k << "^" << L << " vocabulary..." << std::endl;
  voc.create(features);
  std::cout << "... done!" << std::endl;

  std::cout << "Vocabulary information: " << std::endl
  << voc << std::endl << std::endl;

  // lets do something with this vocabulary
  std::cout << "Matching images against themselves (0 low, 1 high): " << std::endl;
  BowVector v1, v2;
  int NIMAGES = features.size();
  vpair.clear();
  for(int i = 0; i < NIMAGES; i++)
  {
		voc.transform(features[i], v1);
		std::cout << "matching " << i << endl;
		for (int j = 0; j < NIMAGES; j++)
		{
			voc.transform(features[j], v2);

			double score = voc.score(v1, v2);
			//if(score > 0.3)
				//  std::cout << "Image " << i << " vs Image " << j << ": " << score << endl;

			if (score > 0.3)
			{
				if (i < j)
				{
					if (i + 20 > j)
					{
						vpair[i] = j;
					}
				}
				else
				{
					if (j + 20 > i)
					{
						vpair[j] = i;
					}
				}
			}
		}
    }

  // save the vocabulary to disk
  //std::cout << endl << "Saving vocabulary..." << endl;
  //voc.save("small_voc.yml.gz");
  //std::cout << "Done" << endl;
}

// ----------------------------------------------------------------------------

void testDatabase(const vector<vector<cv::Mat > > &features)
{
	std::cout << "Creating a small database..." << endl;

  // load the vocabulary from disk
  OrbVocabulary voc("D:\\OpenSourceLib\\ORBSLAM24Windows\\Vocabulary\\ORBvoc.txt.tar.gz");
  
  OrbDatabase db(voc, false, 0); // false = do not use direct index
  // (so ignore the last param)
  // The direct index is useful if we want to retrieve the features that 
  // belong to some vocabulary node.
  // db creates a copy of the vocabulary, we may get rid of "voc" now

  // add images to the database
  int NIMAGES = features.size();
  for(int i = 0; i < NIMAGES; i++)
  {
    db.add(features[i]);
  }

  std::cout << "... done!" << endl;

  std::cout << "Database information: " << endl << db << endl;

  // and query the database
  std::cout << "Querying the database: " << endl;

  QueryResults ret;
  for(int i = 0; i < NIMAGES; i++)
  {
    db.query(features[i], ret, 4);

    // ret[0] is always the same image in this case, because we added it to the 
    // database. ret[1] is the second best match.

	//std::cout << "Searching for Image " << vstrImageFilenames[i] << ". " << ret << endl;
  }

  std::cout << endl;

  // we can save the database. The created file includes the vocabulary
  // and the entries added
  std::cout << "Saving database..." << endl;
  db.save("small_db.yml.gz");
  std::cout << "... done!" << endl;
  
  // once saved, we can load it again  
  std::cout << "Retrieving database once again..." << endl;
  OrbDatabase db2("small_db.yml.gz");
  std::cout << "... done! This is: " << endl << db2 << endl;
}

// ----------------------------------------------------------------------------


