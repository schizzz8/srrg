#include <srrg_system_utils/system_utils.h>
#include <srrg_txt_io/message_reader.h>
#include <srrg_txt_io/message_writer.h>
#include <iostream>
#include <fstream>
#include <limits>
#include <deque>
#include <queue>
#include <vector>

#include <Eigen/Core>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <qapplication.h>
#include <qevent.h>

#include <srrg_system_utils/system_utils.h>
#include <srrg_txt_io/message_reader.h>
#include <srrg_txt_io/message_writer.h>
#include <srrg_txt_io/pinhole_image_message.h>
#include <srrg_txt_io/spherical_image_message.h>
#include <srrg_txt_io/static_transform_tree.h>
#include <srrg_txt_io/message_seq_synchronizer.h>
#include <srrg_nicp/projective_aligner.h>
#include <srrg_nicp_tracker/base_triggers.h>
#include <srrg_nicp_tracker/multi_tracker.h>
#include "srrg_nicp_tracker_viewers/localizer3d_viewer.h"
#include <srrg_core_map/pinhole_camera_info.h>
#include "srrg_core_map/local_map_with_traversability.h"
#include <srrg_boss/deserializer.h>
#include <srrg_boss/trusted_loaders.h>

#include "qapplication.h"
#include <qevent.h>

using namespace std;
using namespace Eigen;
using namespace srrg_core;
using namespace srrg_core_map;
using namespace srrg_nicp;
using namespace srrg_nicp_tracker;
using namespace srrg_nicp_tracker_gui;
using namespace srrg_boss;

Tracker* tracker = 0;
QApplication* app;

typedef std::map<MapNode*,MapNodeSet> MapNodePtrMapNodeSetMap;

const char* banner[]= {
    "fps_tracker_gui_app: offline tracker working on dump files written with fps_message_dumper_node",
    "usage:",
    " fps_tracker_gui_app [options] <dump filename>",
    " where: ",
    "  -aligner:      [string] aligner type [projective or nn], default: projective",
    "  -config:       [string] config type [Xtion640x480, Kinect640x480, Xtion320x240], default: Xtion320x240",
    "  -max_distance: [float] max range of the beams to consider for alignment, default 3",
    "  -min_distance: [float] min range of the beams to consider for alignment, default 0",
    "  -cam_only:     flag, if set ignores the odometry and operates in the camera frame",
    "  -t:            [string] specifies which image topic to use, if unset will use all",
    "                          to issye multiple topics use \"-t <topic1>  -t <topic2> .. -t <topicN> \"",
    "  -rgbt:         [string] specifies which rgb image topics to use. same as above. The number of -rgbt should match the order and the number of -t.",
    "  -single        [flag] if set uses the single projector, otherwise it uses the multi projector on all topics",
    "  -tf            [string] if set overrides the transforms in the file with theones of the file provided",
    "  -bpr:          [float] bad points ratio [float], default: 0.1",
    "  -damping:      [float] solver damping, default: 100",
    "  -shrink :      [int]   image downscaling (2 means half size), default: 1",
    "  -prior :       [string] cloud to load as prior",
    "  -canvas_scale: <float> extends the frustum of the camera to capture larger offsets, default: 1.5",
    "  -max_corr_dist :       [float] maximum distance between corresponding points",
    "  -o:            [string] output filename where to write the model, default \"\"",
    "",
    "Once the gui has started you can:",
    " -dump the current cloud (W key)",
    " -toggle image dumping for movies (D key)",
    " -pause/start the tracker(P key)",
    " -reset the cloud        (R key)",
    0
};

int main(int argc, char **argv) {
    std::string alignerType="projective";
    std::string config="Xtion320x240";
    std::string transforms_filename = "";
    std::string output_filename="";
    std::vector<std::string> depth_topics;
    std::vector<std::string> rgb_topics;
    Eigen::Isometry3f robot_pose = Eigen::Isometry3f::Identity();
    Eigen::Isometry3f current_transform = Eigen::Isometry3f::Identity();
    Eigen::Isometry3f offset = Eigen::Isometry3f::Identity();

    float bad_points_ratio = 0.1;
    float damping = 100;
    int shrink = 1;
    std::string filename = "";
    float min_distance = 0;
    bool cam_only = false;
    int c = 1;
    float max_distance = 3;
    bool single = false;
    string prior="";
    float canvas_scale=1.5;
    float max_correspondence_distance=0.25;
    while (c<argc){
        if (! strcmp(argv[c], "-h")){
            printBanner(banner);
            return 0;
        } else if (! strcmp(argv[c], "-cam_only")){
            cam_only=true;
            cerr << "CAM_ONLY" << endl;
        } else if (! strcmp(argv[c], "-single")){
            single=true;
            cerr << "single tracker" << endl;
        } else if (! strcmp(argv[c], "-aligner")){
            c++;
            alignerType = argv[c];
        } else if (! strcmp(argv[c], "-max_distance")){
            c++;
            max_distance = atof(argv[c]);
        }else if (! strcmp(argv[c], "-max_corr_dist")){
            c++;
            max_correspondence_distance = atof(argv[c]);
        } else if (! strcmp(argv[c], "-canvas_scale")){
            c++;
            canvas_scale = atof(argv[c]);
        } else if (! strcmp(argv[c], "-min_distance")){
            c++;
            min_distance = atof(argv[c]);
        } else if (! strcmp(argv[c], "-t")){
            c++;
            depth_topics.push_back(argv[c]);
        } else if (! strcmp(argv[c], "-rgbt")){
            c++;
            rgb_topics.push_back(argv[c]);
        } else if (! strcmp(argv[c], "-config")){
            c++;
            config = argv[c];
        } else if (! strcmp(argv[c], "-shrink")){
            c++;
            shrink = atoi(argv[c]);
        } else if (! strcmp(argv[c], "-bpr")){
            c++;
            bad_points_ratio = atof(argv[c]);
        } else if (! strcmp(argv[c], "-damping")){
            c++;
            damping = atof(argv[c]);
        } else if (! strcmp(argv[c], "-tf")){
            c++;
            transforms_filename = argv[c];
        } else if (! strcmp(argv[c], "-o")){
            c++;
            output_filename = argv[c];
        } else if (! strcmp(argv[c], "-prior")){
            c++;
            prior = argv[c];
        } else {
            filename = argv[c];
            break;
        }
        c++;
    }
    if (filename.length()==0){
        cerr << "Error: you have to provide an input filename" << endl;
        return 0;
    } else {
        cerr << "reading from file " << filename << endl;
    }

    StaticTransformTree * _transforms = 0;
    if (transforms_filename.length()){
        _transforms = new StaticTransformTree;
        _transforms->load(transforms_filename);
    }


    std::vector<MessageSeqSynchronizer> synchronizers(depth_topics.size());
    if (rgb_topics.size()>0){
        if (rgb_topics.size()!=depth_topics.size()){
            cerr << "fatal error the number of RGB topics should be the same as the -t topics" << endl;
            return 0;
        }
        for (size_t i=0; i<depth_topics.size(); i++){
            std::vector<string> depth_plus_rgb_topic;
            depth_plus_rgb_topic.push_back(depth_topics[i]);
            depth_plus_rgb_topic.push_back(rgb_topics[i]);
            synchronizers[i].setTopics(depth_plus_rgb_topic);
        }
    } else {
        for (size_t i=0; i<depth_topics.size(); i++){
            std::vector<string> depth_topic;
            depth_topic.push_back(depth_topics[i]);
            synchronizers[i].setTopics(depth_topic);
        }
    }

    cerr << "constructing tracker ... ";
    if (depth_topics.size() < 2 || single) {
        tracker = Tracker::makeTracker(alignerType, config);
    } else {
        MultiTracker* multi_tracker = MultiTracker::makeTracker(alignerType, config);
        multi_tracker->init(depth_topics);
        tracker = multi_tracker;
    }
    if (! tracker) {
        cerr << "unknown tracker type [" << alignerType << "] aborting" << endl;
        return 0;
    }
    tracker->setBadPointsRatio(bad_points_ratio);
    tracker->aligner().solver().setDamping(damping);
    //tracker->aligner().baseFinder()->setPointsDistance(max_correspondence_distance);
    tracker->setImageShrink(shrink);
    tracker->setMaxDistance(max_distance);
    tracker->setMinDistance(min_distance);

    ProjectiveAligner* projective_aligner=dynamic_cast<ProjectiveAligner*>(&tracker->aligner());
    if (projective_aligner){
        projective_aligner->setReferenceCanvasScale(canvas_scale);
        cerr << "Setting canvas scale to: " << projective_aligner->referenceCanvasScale() << endl;
    }

    bool first = true;
    MapNodeList nodes;
    BinaryNodeRelationSet edges;
    LocalMapWithTraversability* current_map = new LocalMapWithTraversability;
    MapNodeSet current_neighbors;
    MapNodePtrMapNodeSetMap neighbors_map;
    Cloud transformed_cloud;

    Deserializer des;
    Serializable* o;
    if (prior.length()>0){
        cerr << "Loading prior from file [" << prior << "]" << endl;
        cerr << "tracker set in localization mode" << endl;
        des.setFilePath(prior);

        //Load local maps
        while ( (o = des.readObject()) ){
            LocalMapWithTraversability* lmap = dynamic_cast<LocalMapWithTraversability*>(o);
            if (lmap){
                nodes.addElement(lmap);
                if(lmap->getId() == 3){
                    current_map = lmap;
                    current_transform = current_map->transform();
                    first = false;
                }
            }
            BinaryNodeRelation* rel = dynamic_cast<BinaryNodeRelation*>(o);
            if(rel){
                LocalMapWithTraversability* from = dynamic_cast<LocalMapWithTraversability*>(rel->from());
                LocalMapWithTraversability* to = dynamic_cast<LocalMapWithTraversability*>(rel->to());
                if(from && to)
                    edges.insert(std::tr1::shared_ptr<BinaryNodeRelation>(rel));
            }
            PinholeCameraInfo* info = dynamic_cast<PinholeCameraInfo*>(o);
            if(info)
                offset = info->offset();
        }

        cerr << "Read " << nodes.size() << " local maps." << endl;
        cerr << "Read " << edges.size() << " binary node relations." << endl;

        //Building neighbors map
        for(MapNodeList::iterator it = nodes.begin(); it != nodes.end(); it++)
            for(MapNodeList::iterator jt = nodes.begin(); jt != nodes.end(); jt++)
                if(it->get()->getId() != jt->get()->getId()){
                    int id1 = it->get()->getId(), id2 = jt->get()->getId();
                    for(BinaryNodeRelationSet::iterator kt = edges.begin(); kt != edges.end(); kt++)
                        if((*kt)->from()->getId() == id1 && (*kt)->to()->getId() == id2 ||
                                (*kt)->from()->getId() == id2 && (*kt)->to()->getId() == id1 ){
                            MapNodePtrMapNodeSetMap::iterator lt = neighbors_map.find(it->get());
                            if(lt != neighbors_map.end())
                                lt->second.insert(jt->get());
                            else{
                                MapNodeSet set;
                                set.insert(jt->get());
                                neighbors_map.insert(std::pair<MapNode*,MapNodeSet> (it->get(),set));
                            }
                        }
                }
        current_neighbors = neighbors_map[current_map];
        tracker->enableMerging(false);
        current_map->cloud()->transform(transformed_cloud,current_transform);
        tracker->setReferenceModel(transformed_cloud);
        //tracker->setReferenceModel(*current_map->cloud());
    }

    cerr << " Done" << endl;
    cerr << "opening file " << filename << endl;
    cerr << "ALL IN PLACE" << endl;

    app=new QApplication(argc, argv);

    LocalizerViewer* viewer = new LocalizerViewer(tracker);
    viewer->setTransform(current_transform);
    viewer->show();

    MessageReader reader;
    reader.open(filename);

    BaseMessage* msg;

    bool running = false;
    bool save_snapshot = false;
    ofstream snap_stream;
    int snap_count = 0;
    bool show_changes=0;
    while (reader.good()) {

        if (running) {
            msg = reader.readMessage();
            if (! msg)
                continue;
            BaseImageMessage* base_img=dynamic_cast<BaseImageMessage*>(msg);
            if (! base_img)
                continue;
            Matrix6f odom_info;
            odom_info.setIdentity();
            if (! base_img->hasOdom()){
                odom_info.setZero();
            }
            if (_transforms)
                _transforms->applyTransform(*base_img);

            if(cam_only)
                base_img->setOffset(Eigen::Isometry3f::Identity());
            if(cam_only)
                base_img->setOdometry(Eigen::Isometry3f::Identity());


            BaseImageMessage* base_depth_img=0, *base_rgb_img=0;
            size_t i;
            for (i=0; i<synchronizers.size(); i++){
                synchronizers[i].putMessage(base_img);
                if (synchronizers[i].messagesReady()) {
                    base_depth_img=dynamic_cast<BaseImageMessage*>(synchronizers[i].messages()[0].get());
                    if (synchronizers[i].messages().size()>1)
                        base_rgb_img=dynamic_cast<BaseImageMessage*>(synchronizers[i].messages()[1].get());
                    break;
                }
            }
            if (! base_depth_img)
                continue;
            RGBImage rgb_image;
            if (base_rgb_img)
                rgb_image =base_rgb_img->image();


            PinholeImageMessage* pinhole_depth_img=dynamic_cast<PinholeImageMessage*>(base_depth_img);
            PinholeImageMessage* pinhole_rgb_img=dynamic_cast<PinholeImageMessage*>(base_rgb_img);

            SphericalImageMessage* spherical_depth_img=dynamic_cast<SphericalImageMessage*>(base_depth_img);
            SphericalImageMessage* spherical_rgb_img=dynamic_cast<SphericalImageMessage*>(base_rgb_img);

            if (pinhole_depth_img){
                tracker->processFrame(pinhole_depth_img->image(),
                                      rgb_image,
                                      pinhole_depth_img->cameraMatrix(),
                                      pinhole_depth_img->depthScale(),
                                      pinhole_depth_img->seq(),
                                      pinhole_depth_img->timestamp(),
                                      pinhole_depth_img->topic(),
                                      pinhole_depth_img->frameId(),
                                      pinhole_depth_img->offset(),
                                      pinhole_depth_img->odometry(),
                                      odom_info);
            }

            robot_pose = current_transform.inverse()*tracker->globalT();
            if(! current_map->isTraversable(robot_pose.translation()))
                for(MapNodeSet::iterator it = current_neighbors.begin(); it != current_neighbors.end(); it++){
                    LocalMapWithTraversability* neighbor = dynamic_cast<LocalMapWithTraversability*>(*it);
                    Eigen::Vector3f transformed_position = (neighbor->transform().inverse()*tracker->globalT()).translation();
                    if(neighbor->isTraversable(transformed_position)){
                        current_map = neighbor;
                        current_transform = current_map->transform();
                        current_neighbors = neighbors_map[current_map];
                        cerr << "ID: " << current_map->getId() << endl;
                        cerr << "Transform: " << current_transform.translation().transpose() << endl;
                        tracker->enableMerging(false);
                        current_map->cloud()->transform(transformed_cloud,tracker->globalT().inverse()*current_transform);
                        tracker->setReferenceModel(transformed_cloud);
                        viewer->setTransform(current_transform);
                        break;
                    }
                }
            if (tracker->isTrackBroken()){
                tracker->clearStatus();
            }
            viewer->updateGL();
            app->processEvents();
            synchronizers[i].reset();

            if (save_snapshot) {
                if (! snap_stream ) {
                    snap_stream.open("images.txt");
                }
                char snap_filename[100];
                sprintf(snap_filename,"image-%07d.jpg", snap_count);
                snap_stream << snap_filename << endl;
                viewer->saveSnapshot(QString(snap_filename));
                snap_count ++;
            }

        } else {
            app->processEvents();
            usleep(20000);
        }
        QKeyEvent* event=viewer->lastKeyEvent();
        if (event){
            switch(event->key()) {
            case Qt::Key_R:
                tracker->clearStatus();
                break;
            case Qt::Key_1:
                viewer->setFollowCamera(!viewer->followCamera());
                break;
            case Qt::Key_D:
                save_snapshot = ! save_snapshot;
                cerr << "snapshot now is " << save_snapshot << endl;
                break;
            case Qt::Key_P:
                running = ! running;
                break;
            case Qt::Key_B:
                reader = MessageReader();
                reader.open(filename);
                tracker->clearStatus();
                break;
            case Qt::Key_M:
                tracker->enableMerging(!tracker->mergingEnabled());
                break;
            case Qt::Key_J:
                show_changes=!show_changes;
                if (show_changes) {
                    tracker->setChangesThreshold(0.05);
                } else {
                    tracker->setChangesThreshold(0);
                }
            default:;
            }
            viewer->keyEventProcessed();
        }
    }
    app->exec();

}
