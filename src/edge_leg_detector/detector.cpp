// FILE: "Leg_detector_node" 
// AUTHOR: Marco Antonio Becerra Pedraza (http://www.marcobecerrap.com)
// MODIFIED BY: Ferdian Jovan (http://github.com/ferdianjovan)
// SUMMARY: This program receives the LaserScan msgs and executes a leg detector algorithm
// > to search for persons. At the end publishes a a vector with all the persons found 
// > and their position relative to the sensor.
//
// NOTES: This leg detector is based on the work described in:
// Bellotto, N. & Hu, H. 
// Multisensor-Based Human Detection and Tracking for Mobile Service Robots 
// IEEE Trans. on Systems, Man, and Cybernetics -- Part B, 2009, 39, 167-181

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <list>
#include <string>
#include <boost/thread/mutex.hpp>

#define PI 3.1416
#define FILTER_SIZE 2
#define FLANK_THRESHOLD 0.1

#define FLANK_U 1
#define FLANK_D -1

//Antropometric parameters
#define ANTRO_a0 0.1 //|
#define ANTRO_a1 0.2 //|-> Leg width (min-max)
#define ANTRO_b0 0   //  |
#define ANTRO_b1 0.4 //  |-> Free space between two legs (min-max)
#define ANTRO_c0 0.1 //    |
#define ANTRO_c1 0.4 //    |-> Two legs together width (min-max)

// Pattern Type
#define TYPE_LA 1 // Legs separated
#define TYPE_FS 2 // Legs dephased
#define TYPE_SL 3 // Legs together


using namespace std;

bool sensor_on   = false;

int g_counter = 0;

vector < double > rec_x;
vector < double > rec_y;
string sensor_frame_id;
sensor_msgs::LaserScan SensorMsg;
boost::mutex mutex;

void LaserFilter_Mean( vector <double> *vector_r, unsigned size );
void LaserCallback (const sensor_msgs::LaserScan::ConstPtr& msg);
void FindPattern( string str, string pattern, list <int> *element_found );
void ValidatePattern( list <int> *Pattern_list, int TYPE,  vector <int> flank_id0,  vector <int> flank_id1, vector <double> laser_x, vector <double> laser_y);
double Dist2D( double x0, double y0, double x1, double y1 );
void HumanPose( vector <double> *r_x, vector <double> *r_y, list <int> Pattern_list, int TYPE,  vector <int> flank_id0,  vector <int> flank_id1, vector <double> laser_x, vector <double> laser_y );

// added by Ferdian Jovan
// function to restrict a possibility of persons standing next to each other
void ValidateDistance();


int main(int argc, char **argv){

  ros::init(argc, argv, "edge_leg_detector");
  ros::NodeHandle n;
  ros::Publisher  node_pub = n.advertise <geometry_msgs::PoseArray>("edge_leg_detector", 2); // Humans in the environment

  // get param from launch file
  string laser_scan = "/scan";
  ros::param::get("~laser_scan", laser_scan);
  ros::Subscriber node_sub = n.subscribe(laser_scan, 2, LaserCallback);
  geometry_msgs::PoseArray msgx;
  ros::Rate loop_rate(15);


  int seq_counter = 0;
  
  while( ros::ok() ){
    if( sensor_on == true ){

      // delete persons who are too near to each other
      void ValidateDistance();

      //------------------------------------------
      // Copying to proper PoseArray data structure
      vector < geometry_msgs::Pose > HumanPoseVector;
      for( int K = 0; K < rec_x.size(); K++ ){
	geometry_msgs::Point HumanPoint;
	geometry_msgs::Quaternion HumanQuaternion;
	
	HumanPoint.x = rec_x[ K ];
	HumanPoint.y = rec_y[ K ];
	HumanPoint.z = 0; 
	
	HumanQuaternion.x = 0;//|-> Orientation is ignored
	HumanQuaternion.y = 0;//|
	HumanQuaternion.z = 0;//|
	HumanQuaternion.w = 1;//|

	geometry_msgs::Pose HumanPose;
	HumanPose.position = HumanPoint;
	HumanPose.orientation= HumanQuaternion;
	HumanPoseVector.push_back( HumanPose );
      }

      // Header config
      msgx.header.stamp = ros::Time::now();
      msgx.header.frame_id = SensorMsg.header.frame_id;
      msgx.header.seq = seq_counter;
      msgx.poses = HumanPoseVector;
      //------------------------------------------

      node_pub.publish( msgx );
    }
    ros::spinOnce();
    loop_rate.sleep();
    seq_counter++;
  }

  return 0;
}


void LaserCallback (const sensor_msgs::LaserScan::ConstPtr& msg){

  // To get header data from sensor msg
  SensorMsg = *msg;

  // Vectors...
  rec_x.clear(); 
  rec_y.clear(); 
  
  sensor_on = true;
  
  double px, py, pr, pt;
  vector < double >  laser_x;
  vector < double >  laser_y;
  vector < double >  laser_r;
  vector < double >  laser_t;
  for( unsigned i = 0; i < msg->ranges.size(); i++ ){    
    pr = msg->ranges[ i ];
    pt = msg->angle_min + ( i * msg->angle_increment);
    laser_r.push_back( pr );
    laser_t.push_back( pt );
  }
  
  // Filtering laser scan
  LaserFilter_Mean( &laser_r, FILTER_SIZE );
  for( unsigned i = 0; i < msg->ranges.size(); i++ ){    
    px = laser_r[ i ] * cos( laser_t[ i ] );
    py = laser_r[ i ] * sin( laser_t[ i ] );
    laser_x.push_back( px );
    laser_y.push_back( py );
  }
	 
  string str_aux = "";
  // Finding flanks in the laser scan...
  vector < int > laser_flank;
  laser_flank.assign(laser_r.size(), 0);
  for( unsigned i = 1; i < laser_flank.size(); i++ ){
    if( fabs( laser_r[ i ] - laser_r[ i - 1 ] ) > FLANK_THRESHOLD )
      laser_flank[ i ] = ( ( laser_r[ i ] - laser_r[ i - 1 ] ) > 0 ) ? FLANK_U : FLANK_D;
  }
  
  vector < int >    flank_id0;
  vector < int >    flank_id1;
  string flank_string = "";
  int past_value = 0;
  int idx = 0;
  for( unsigned i = 1; i < laser_flank.size(); i++ ){
    if( laser_flank[ i ] != 0 ){
      if( past_value != laser_flank[ i ] ){
	flank_id0.push_back( i - 1 );
	flank_id1.push_back( i );
	flank_string += ( laser_flank[ i ] > 0 ) ? "S" : "B";
	idx++;
      }
      else
      	flank_id1[ idx - 1 ] =  i;    
    }
    past_value = laser_flank[ i ];
  }  
    
  // PATTERN RECOGNITION
  string LEGS_LA  = "BSBS";
  string LEGS_FS1 = "BBS";
  string LEGS_FS2 = "BSS";
  string LEGS_SL = "BS";
  
  list <int> Pattern_LA;
  list <int> Pattern_FS1;
  list <int> Pattern_FS2;
  list <int> Pattern_SL;
 
  FindPattern( flank_string, LEGS_LA,  &Pattern_LA  );
  FindPattern( flank_string, LEGS_FS1, &Pattern_FS1 );
  FindPattern( flank_string, LEGS_FS2, &Pattern_FS2 );
  FindPattern( flank_string, LEGS_SL,  &Pattern_SL  );  

  // ANTROPOMETRIC VALIDATION (the non antropometric patterns are erased from the list)
  ValidatePattern( &Pattern_LA,  TYPE_LA, flank_id0, flank_id1,  laser_x, laser_y);
  ValidatePattern( &Pattern_FS1, TYPE_FS, flank_id0, flank_id1,  laser_x, laser_y);
  ValidatePattern( &Pattern_FS2, TYPE_FS, flank_id0, flank_id1,  laser_x, laser_y);
  ValidatePattern( &Pattern_SL,  TYPE_SL, flank_id0, flank_id1,  laser_x, laser_y);

  // ERASE REDUNDANT PATTERNS FROM ACCEPTED ONES (If a LA or FS pattern is accepted, we erase the SL on it)
  // a) Erase SL from LA
  list<int>::iterator it_K;
  for( it_K = Pattern_LA.begin(); it_K != Pattern_LA.end(); it_K++ ){
    list<int>::iterator it_M;
    // Erase first leg
    for( it_M = Pattern_SL.begin(); it_M != Pattern_SL.end(); it_M++ )
      if( flank_id0[ *it_K ] == flank_id0[ *it_M ] ){
	Pattern_SL.erase( it_M );
	break;
      }
    // Erase second leg
    for( it_M = Pattern_SL.begin(); it_M != Pattern_SL.end(); it_M++ )
      if( flank_id0[ *it_K + 2 ] == flank_id0[ *it_M ] ){
    	Pattern_SL.erase( it_M );
    	break;
      }

  }
  // b) Erase SL from FS1 "BBS"
  for( it_K = Pattern_FS1.begin(); it_K != Pattern_FS1.end(); it_K++ ){
    list<int>::iterator it_M;
    for( it_M = Pattern_SL.begin(); it_M != Pattern_SL.end(); it_M++ )
      if( flank_id0[ *it_K + 1 ] == flank_id0[ *it_M ] ){
  	Pattern_SL.erase( it_M );
  	break;
      }
  }
  // c) Erase SL from FS2 "BSS"
  for( it_K = Pattern_FS1.begin(); it_K != Pattern_FS1.end(); it_K++ ){
    list<int>::iterator it_M;
    for( it_M = Pattern_SL.begin(); it_M != Pattern_SL.end(); it_M++ )
      if( flank_id0[ *it_K ] == flank_id0[ *it_M ] ){
  	Pattern_SL.erase( it_M );
  	break;
      }
  }

  
  boost::mutex::scoped_lock lock(mutex);
  //CENTROID PATTERN COMPUTATION & UNCERTAINTY
  rec_x.clear();
  rec_y.clear();
  
  HumanPose( &rec_x, &rec_y, Pattern_LA,  TYPE_LA,  flank_id0, flank_id1,  laser_x, laser_y);
  HumanPose( &rec_x, &rec_y, Pattern_FS1, TYPE_FS,  flank_id0, flank_id1,  laser_x, laser_y);
  HumanPose( &rec_x, &rec_y, Pattern_FS2, TYPE_FS,  flank_id0, flank_id1,  laser_x, laser_y);
  HumanPose( &rec_x, &rec_y, Pattern_SL,  TYPE_SL,  flank_id0, flank_id1,  laser_x, laser_y);
}


// Mean value of the 'size' adjacent values
void LaserFilter_Mean( vector <double> *vector_r, unsigned size ){
  for( unsigned i = 0; i < ( (*vector_r).size() - size ); i++ ){
      double mean = 0;
      for( unsigned k = 0; k < size; k++  ){
	mean += (*vector_r)[ i + k ];
      }
      (*vector_r)[ i ] = mean / size;
  }
}


// Reports a found string pattern in a list
void FindPattern( string str, string pattern, list <int> *element_found ){
  size_t found = 0;

  while( string::npos != ( found = str.find( pattern, found ) ) ){
    (*element_found).push_back( found ); 
    found++;
  }
  
} 


// Performs the antropometric validation of the leg patterns
void ValidatePattern( list <int> *Pattern_list, int TYPE,  vector <int> flank_id0,  vector <int> flank_id1, vector <double> laser_x, vector <double> laser_y){
  
  double ANTRO_a_1, ANTRO_a_2, ANTRO_b, ANTRO_c; // Antropometric values from patterns to compare with constants.
  bool SavePattern = true;
  bool cond_a = true, cond_b = true, cond_c = true;
  list<int>::iterator it;
  
  for( it = (*Pattern_list).begin(); it != (*Pattern_list).end(); it++ ){

    // Obtain antropometric values
    switch( TYPE ){
      case TYPE_LA: //BSBS
	ANTRO_a_1 = Dist2D( laser_x[ flank_id1[ *it ] ], laser_y[ flank_id1[ *it ] ], laser_x[ flank_id0[ *it + 1 ] ], laser_y[ flank_id0[ *it + 1 ] ]);
	ANTRO_a_2 = Dist2D( laser_x[ flank_id1[ *it + 2 ] ], laser_y[ flank_id1[ *it + 2 ] ], laser_x[ flank_id0[ *it + 3 ] ], laser_y[ flank_id0[ *it + 3 ] ]);
	ANTRO_b = Dist2D( laser_x[ flank_id0[ *it + 1 ] ], laser_y[ flank_id0[ *it + 1 ] ], laser_x[ flank_id1[ *it + 2 ] ], laser_y[ flank_id1[ *it + 2 ] ] );
	ANTRO_c = 0;
	cond_a = ( ( ANTRO_a_1 >= ANTRO_a0 ) && ( ANTRO_a_1 <= ANTRO_a1 ) ) && ( ( ANTRO_a_2 >= ANTRO_a0 ) && ( ANTRO_a_2 <= ANTRO_a1 ) );
	cond_b = ( ( ANTRO_b >= ANTRO_b0 ) && ( ANTRO_b <= ANTRO_b1 ) );
	cond_c = true;
        break;
      case TYPE_FS: // BBS & BSS
	ANTRO_a_1 = Dist2D( laser_x[ flank_id1[ *it ] ], laser_y[ flank_id1[ *it ] ], laser_x[ flank_id0[ *it + 1 ] ], laser_y[ flank_id0[ *it + 1 ] ]);
	ANTRO_a_2 = Dist2D( laser_x[ flank_id1[ *it + 1 ] ], laser_y[ flank_id1[ *it + 1 ] ], laser_x[ flank_id0[ *it + 2 ] ], laser_y[ flank_id0[ *it + 2 ] ]);
	ANTRO_b = Dist2D( laser_x[ flank_id0[ *it + 1 ] ], laser_y[ flank_id0[ *it + 1 ] ], laser_x[ flank_id1[ *it + 1 ] ], laser_y[ flank_id1[ *it + 1 ] ] );
	ANTRO_c = 0;
	cond_a = ( ( ANTRO_a_1 >= ANTRO_a0 ) && ( ANTRO_a_1 <= ANTRO_a1 ) ) && ( ( ANTRO_a_2 >= ANTRO_a0 ) && ( ANTRO_a_2 <= ANTRO_a1 ) );
	cond_b = ( ( ANTRO_b >= ANTRO_b0 ) && ( ANTRO_b <= ANTRO_b1 ) );
	cond_c = true;
        break;
    case TYPE_SL: // BS
      	ANTRO_a_1 = 0;
	ANTRO_a_2 = 0;
	ANTRO_b = 0;
	ANTRO_c = Dist2D( laser_x[ flank_id1[ *it ] ], laser_y[ flank_id1[ *it ] ], laser_x[ flank_id0[ *it + 1 ] ], laser_y[ flank_id0[ *it + 1 ] ]);
	cond_a = true;
	cond_b = true;	
	cond_c = ( ( ANTRO_c >= ANTRO_c0 ) && ( ANTRO_c <= ANTRO_c1 ) );
	break;
    }

    SavePattern = cond_a && cond_b && cond_c;
    
    if( !SavePattern ){
      it = (*Pattern_list).erase( it );
      it--;
    }
  }  
}


// Euclidean distance between two coordinate points
double Dist2D( double x0, double y0, double x1, double y1 ){
  return sqrt( pow( x0 - x1, 2 ) + pow( y0 - y1, 2 ) );
}


void HumanPose( vector <double> *r_x, vector <double> *r_y, list <int> Pattern_list, int TYPE,  vector <int> flank_id0,  vector <int> flank_id1, vector <double> laser_x, vector <double> laser_y ){
  
  double c_x, c_y;
  int l1, l2, l3, l4;
  int count; 
  list<int>::iterator it;

  for( it = Pattern_list.begin(); it != Pattern_list.end(); it++ ){
    c_x = 0;
    c_y = 0;
    count = 0;

    l1 = flank_id1[ *it ];
    l2 = flank_id0[ *it + 1 ];
    
    switch( TYPE ){
    case TYPE_LA:
      l3 = flank_id1[ *it + 2 ];
      l4 = flank_id0[ *it + 3 ];
      break;
    case TYPE_FS:
      l3 = flank_id1[ *it + 1 ];
      l4 = flank_id0[ *it + 2 ];
      break;
    case TYPE_SL:
      l3 = 1;
      l4 = 0;
      break;
    }

    for( int i = l1; i <= l2; i++ ){
      c_x += laser_x[ i ];
      c_y += laser_y[ i ];
      count++;
    }
    for( int i = l3; i <= l4; i++ ){
      c_x += laser_x[ i ];
      c_y += laser_y[ i ];
      count++;
    }
    
    c_x /= (double) count;
    c_y /= (double) count;
    
    (*r_x).push_back( c_x );
    (*r_y).push_back( c_y );
  }
}


// Validate distance between persons
void ValidateDistance(){
    boost::mutex::scoped_lock lock(mutex);
    int j = 0;
    while(j < (rec_x.size() - 1))
    {
        // if the Euclidean distance between two persons are smaller than
        // the maximum width of a leg then the second person must be eliminated
        if (ANTRO_b1 > Dist2D(rec_x[j], rec_y[j], rec_x[j+1], rec_y[j+1]))
        {
            rec_x.erase(rec_x.begin() + (j + 1));
            rec_y.erase(rec_y.begin() + (j + 1));
        }
        else
        {
            j++;
        }
    }
}
