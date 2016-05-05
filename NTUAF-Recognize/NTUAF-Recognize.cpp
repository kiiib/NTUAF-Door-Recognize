// NTUAF-Recognize.cpp : 定義主控台應用程式的進入點。
//

#include "stdafx.h"
#include <Kinect.h>
#include <opencv2\core.hpp>
#include <opencv2\imgproc.hpp>
#include <opencv2\highgui.hpp>
#include <iostream>
#include <time.h>
#include <fstream>
#include <stdlib.h>  //使用exit須載入stdlib標頭檔

using namespace std;

void DrawLine(cv::Mat& rImg, const Joint& rJ1, const Joint& rJ2, ICoordinateMapper* pCMapper, string nowTime)
{
	if (rJ1.TrackingState == TrackingState_NotTracked || rJ2.TrackingState == TrackingState_NotTracked)
		return;

	ColorSpacePoint ptJ1, ptJ2;
	pCMapper->MapCameraPointToColorSpace(rJ1.Position, &ptJ1);
	pCMapper->MapCameraPointToColorSpace(rJ2.Position, &ptJ2);

	if (rJ2.JointType == JointType_Head) {
		cout << "X: " << ptJ1.X << endl;
		cout << "Y: " << ptJ1.Y << endl;

		fstream file;

		char *str[2] = { "X","Y" };  //宣告字串指標陣列

		float id[2] = { ptJ1.X,ptJ1.Y };

		file.open("data.txt", ios::app);      //開啟檔案

		if (!file)     //檢查檔案是否成功開啟

		{
			cerr << "Can't open file!\n";
			exit(1);     //在不正常情形下，中斷程式的執行

		}
		file << nowTime << ".jpeg" << "\n";
		for (int i = 0; i < 2; i++) {
			file << str[i] << ":" << id[i] << "\n";
		}      //將資料輸出至檔案
	}

	cv::line(rImg, cv::Point(ptJ1.X, ptJ1.Y), cv::Point(ptJ2.X, ptJ2.Y), cv::Vec3b(10, 230, 30), 5);

}

int main(int argc, char** argv)
{


	// 1a. Get default Sensor
	cout << "Try to get default sensor" << endl;
	IKinectSensor* pSensor = nullptr;
	if (GetDefaultKinectSensor(&pSensor) != S_OK)
	{
		cerr << "Get Sensor failed" << endl;
		return -1;
	}

	// 1b. Open sensor
	cout << "Try to open sensor" << endl;
	if (pSensor->Open() != S_OK)
	{
		cerr << "Can't open sensor" << endl;
		return -1;
	}

	// 2. Color Related code
	IColorFrameReader* pColorFrameReader = nullptr;
	cv::Mat	mColorImg;
	UINT uBufferSize = 0;
	{
		// 2a. Get color frame source
		cout << "Try to get color source" << endl;
		IColorFrameSource* pFrameSource = nullptr;
		if (pSensor->get_ColorFrameSource(&pFrameSource) != S_OK)
		{
			cerr << "Can't get color frame source" << endl;
			return -1;
		}

		// 2b. Get frame description
		cout << "get color frame description" << endl;
		int		iWidth = 0;
		int		iHeight = 0;
		IFrameDescription* pFrameDescription = nullptr;
		if (pFrameSource->get_FrameDescription(&pFrameDescription) == S_OK)
		{
			pFrameDescription->get_Width(&iWidth);
			pFrameDescription->get_Height(&iHeight);
		}
		pFrameDescription->Release();
		pFrameDescription = nullptr;

		// 2c. get frame reader
		cout << "Try to get color frame reader" << endl;
		if (pFrameSource->OpenReader(&pColorFrameReader) != S_OK)
		{
			cerr << "Can't get color frame reader" << endl;
			return -1;
		}

		// 2d. release Frame source
		cout << "Release frame source" << endl;
		pFrameSource->Release();
		pFrameSource = nullptr;

		// Prepare OpenCV data
		mColorImg = cv::Mat(iHeight, iWidth, CV_8UC4);
		uBufferSize = iHeight * iWidth * 4 * sizeof(BYTE);


		//cout << "mColorImg = " << endl << " " << mColorImg << endl;
	}



	// 3. Body related code
	IBodyFrameReader* pBodyFrameReader = nullptr;
	IBody** aBodyData = nullptr;
	INT32 iBodyCount = 0;
	{
		// 3a. Get frame source
		cout << "Try to get body source" << endl;
		IBodyFrameSource* pFrameSource = nullptr;
		if (pSensor->get_BodyFrameSource(&pFrameSource) != S_OK)
		{
			cerr << "Can't get body frame source" << endl;
			return -1;
		}

		// 3b. Get the number of body
		if (pFrameSource->get_BodyCount(&iBodyCount) != S_OK)
		{
			cerr << "Can't get body count" << endl;
			return -1;
		}
		cout << " > Can trace " << iBodyCount << " bodies" << endl;
		aBodyData = new IBody*[iBodyCount];
		for (int i = 0; i < iBodyCount; ++i)
			aBodyData[i] = nullptr;

		// 3c. get frame reader
		cout << "Try to get body frame reader" << endl;
		if (pFrameSource->OpenReader(&pBodyFrameReader) != S_OK)
		{
			cerr << "Can't get body frame reader" << endl;
			return -1;
		}

		// 3d. release Frame source
		cout << "Release frame source" << endl;
		pFrameSource->Release();
		pFrameSource = nullptr;
	}

	// 4. get CoordinateMapper
	ICoordinateMapper* pCoordinateMapper = nullptr;
	if (pSensor->get_CoordinateMapper(&pCoordinateMapper) != S_OK)
	{
		cout << "Can't get coordinate mapper" << endl;
		return -1;
	}

	// Enter main loop
	cv::namedWindow("Body Image");
	int countTime = 0;
	int picNum = 0;
	char t[1024];
	string strPicNum;
	vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
	compression_params.push_back(95);

	while (true)
	{
		time_t outTime;
		// 4a. Get last frame
		IColorFrame* pColorFrame = nullptr;
		if (pColorFrameReader->AcquireLatestFrame(&pColorFrame) == S_OK)
		{
			// 4c. Copy to OpenCV image
			if (pColorFrame->CopyConvertedFrameDataToArray(uBufferSize, mColorImg.data, ColorImageFormat_Bgra) != S_OK)
			{
				cerr << "Data copy error" << endl;
			}

			// 4e. release frame
			pColorFrame->Release();
		}
		cv::Mat mImg = mColorImg.clone();

		//map 


		// 4b. Get body data
		IBodyFrame* pBodyFrame = nullptr;
		//float distance;

		if (pBodyFrameReader->AcquireLatestFrame(&pBodyFrame) == S_OK)
		{
			// 4b. get Body data
			if (pBodyFrame->GetAndRefreshBodyData(iBodyCount, aBodyData) == S_OK)
			{
				// 4c. for each body
				for (int i = 0; i < iBodyCount; ++i)
				{


					IBody* pBody = aBodyData[i];

					// check if is tracked
					BOOLEAN bTracked = false;
					if ((pBody->get_IsTracked(&bTracked) == S_OK) && bTracked)
					{
						// get joint position
						Joint aJoints[JointType::JointType_Count];
						if (pBody->GetJoints(JointType::JointType_Count, aJoints) == S_OK)
						{

							/*cout << "Left Shoulder Position X: " << aJoints[JointType_ShoulderRight].Position.X << endl;
							cout << "Wrist Left Position X: " << aJoints[JointType_WristLeft].Position.X << endl;
							cout << "Left Shoulder Position Y: " << aJoints[JointType_ShoulderRight].Position.Y << endl;
							cout << "Wrist Left Position Y: " << aJoints[JointType_WristLeft].Position.Y << endl;
							cout << "Left Shoulder Position Z: " << aJoints[JointType_ShoulderRight].Position.Z << endl;
							cout << "Wrist Left Position Z: " << aJoints[JointType_WristLeft].Position.Z<< endl;
							cout << "Left Shoulder Position JointType: " << aJoints[JointType_Head].JointType << endl;
							cout << "Wrist Left Position JointType: " << aJoints[JointType_WristLeft].JointType << endl;*/

							//float disFromHandLtoHanR = sqrt(pow(aJoints[JointType_HandLeft].Position.X - aJoints[JointType_HandRight].Position.X, 2) + pow(aJoints[JointType_HandLeft].Position.Y - aJoints[JointType_HandRight].Position.Y, 2) + pow(aJoints[JointType_HandLeft].Position.Z - aJoints[JointType_HandRight].Position.Z, 2));
							//cout << "disFromHandLtoHanR: " << disFromHandLtoHanR << endl;

							//float disFromHeadLtoFoot = sqrt(pow(aJoints[JointType_Head].Position.X - aJoints[JointType_FootRight].Position.X, 2) + pow(aJoints[JointType_Head].Position.Y - aJoints[JointType_FootRight].Position.Y, 2) + pow(aJoints[JointType_Head].Position.Z - aJoints[JointType_FootRight].Position.Z, 2));
							//cout << "disFromHeadLtoFoot: " << disFromHeadLtoFoot << endl;

							//cout << disFromHeadLtoFoot << endl;
							/*out << aJoints[JointType_Head].Position.X << "     " << aJoints[JointType_Head].Position.Y;*/
							//float head_x = aJoints[JointType_Head].Position.X ;
							//std::string headX = std::to_string(head_x);

							//fwrite(headX, sizeof(char), sizeof(headX), pFile);




							countTime++;

							cout << "countFrame = " << countTime << endl;
							if (countTime == 45) {
								/*DrawLine(mImg, aJoints[JointType_SpineBase], aJoints[JointType_SpineMid], pCoordinateMapper);
								DrawLine(mImg, aJoints[JointType_SpineMid], aJoints[JointType_SpineShoulder], pCoordinateMapper);
								DrawLine(mImg, aJoints[JointType_SpineShoulder], aJoints[JointType_Neck], pCoordinateMapper);*/
								/*DrawLine(mImg, aJoints[JointType_Neck], aJoints[JointType_Head], pCoordinateMapper);*/

								/*DrawLine(mImg, aJoints[JointType_SpineShoulder], aJoints[JointType_ShoulderLeft], pCoordinateMapper);
								DrawLine(mImg, aJoints[JointType_ShoulderLeft], aJoints[JointType_ElbowLeft], pCoordinateMapper);
								DrawLine(mImg, aJoints[JointType_ElbowLeft], aJoints[JointType_WristLeft], pCoordinateMapper);
								DrawLine(mImg, aJoints[JointType_WristLeft], aJoints[JointType_HandLeft], pCoordinateMapper);
								DrawLine(mImg, aJoints[JointType_HandLeft], aJoints[JointType_HandTipLeft], pCoordinateMapper);
								DrawLine(mImg, aJoints[JointType_HandLeft], aJoints[JointType_ThumbLeft], pCoordinateMapper);

								DrawLine(mImg, aJoints[JointType_SpineShoulder], aJoints[JointType_ShoulderRight], pCoordinateMapper);
								DrawLine(mImg, aJoints[JointType_ShoulderRight], aJoints[JointType_ElbowRight], pCoordinateMapper);
								DrawLine(mImg, aJoints[JointType_ElbowRight], aJoints[JointType_WristRight], pCoordinateMapper);
								DrawLine(mImg, aJoints[JointType_WristRight], aJoints[JointType_HandRight], pCoordinateMapper);
								DrawLine(mImg, aJoints[JointType_HandRight], aJoints[JointType_HandTipRight], pCoordinateMapper);
								DrawLine(mImg, aJoints[JointType_HandRight], aJoints[JointType_ThumbRight], pCoordinateMapper);

								DrawLine(mImg, aJoints[JointType_SpineBase], aJoints[JointType_HipLeft], pCoordinateMapper);
								DrawLine(mImg, aJoints[JointType_HipLeft], aJoints[JointType_KneeLeft], pCoordinateMapper);
								DrawLine(mImg, aJoints[JointType_KneeLeft], aJoints[JointType_AnkleLeft], pCoordinateMapper);
								DrawLine(mImg, aJoints[JointType_AnkleLeft], aJoints[JointType_FootLeft], pCoordinateMapper);

								DrawLine(mImg, aJoints[JointType_SpineBase], aJoints[JointType_HipRight], pCoordinateMapper);
								DrawLine(mImg, aJoints[JointType_HipRight], aJoints[JointType_KneeRight], pCoordinateMapper);
								DrawLine(mImg, aJoints[JointType_KneeRight], aJoints[JointType_AnkleRight], pCoordinateMapper);
								DrawLine(mImg, aJoints[JointType_AnkleRight], aJoints[JointType_FootRight], pCoordinateMapper);*/

								int newTime;
								newTime = time(&outTime);
								picNum++;
								cv::Mat mat(800, 600, CV_8UC4);
								mat = mImg.clone();

								sprintf_s(t, "%d", newTime);
								strPicNum = t;
								string str = "./images/";
								cout << "picpath : " << str << strPicNum << ".jpeg" << endl;
								try {
									DrawLine(mImg, aJoints[JointType_Neck], aJoints[JointType_Head], pCoordinateMapper, strPicNum);
									cv::imwrite(str + strPicNum + ".jpeg", mat, compression_params);
								}
								catch (cv::Exception& ex) {
									fprintf(stderr, "Exception converting image to JPG format: %s\n", ex.what());
								}
								fprintf(stdout, "Saved JPG file.\n");
								mat.release();
								countTime = 0;
							}
						}

					}
				}
			}
			else
			{
				cerr << "Can't read body data" << endl;
			}
			// 4e. release frame
			pBodyFrame->Release();
		}

		// show image
		//		cv::imwrite("haha.jpg", mImg);
		cv::imshow("Body Image", mImg);

		// 4c. check keyboard input
		if (cv::waitKey(30) == VK_ESCAPE) {
			break;
		}
	}

	// 3. delete body data array
	delete[] aBodyData;

	// 3. release frame reader
	cout << "Release body frame reader" << endl;
	pBodyFrameReader->Release();
	pBodyFrameReader = nullptr;

	// 2. release color frame reader
	cout << "Release color frame reader" << endl;
	pColorFrameReader->Release();
	pColorFrameReader = nullptr;

	// 1c. Close Sensor
	cout << "close sensor" << endl;
	pSensor->Close();

	// 1d. Release Sensor
	cout << "Release sensor" << endl;
	pSensor->Release();
	pSensor = nullptr;

	return 0;
}
