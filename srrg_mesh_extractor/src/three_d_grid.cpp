#include "three_d_grid.h"

using namespace std;

namespace std {
template<>
bool std::less<Eigen::Vector3i>::operator ()(const Eigen::Vector3i& a,const Eigen::Vector3i& b) const {
    for(size_t i=0;i<3;++i) {
        if(a[i]<b[i]) return true;
        if(a[i]>b[i]) return false;
    }
    return false;
}
}

BaseGrid::BaseGrid(const PointCloud::Ptr &cloud, int prec) : _cloud(new pcl::PointCloud<PointType>){
    _cloud = cloud;
    double res = computeCloudResolution();
    PointType min_pt,max_pt;
    pcl::getMinMax3D(*_cloud,min_pt,max_pt);

    cout << "\n";
    cout << BOLD(FBLU("Loading the Data-Set:\n"));
    cout << "\t>> Points: " << _cloud->size() << "\n";
    cout << "\t>> Min: (" << min_pt.x << "," << min_pt.y << "," << min_pt.z << ")\n";
    cout << "\t>> Max: (" << max_pt.x << "," << max_pt.y << "," << max_pt.z << ")\n";
    cout << "\t>> Average distance: " << res << "m\n";
    cout << "--------------------------------------------------------------------------------\n";
    cout << "\n";

    bool found = false;
    int i = 2;
    while(found == false)
    {
        i--;
        if(getdigit(res,i) != 0)
            found = true;
    }
    _resolution = (100/prec)*pow(10,i);
    _inverse_resolution = 1./_resolution;

    _origin.x() = min_pt.x - 5*_resolution;
    _origin.y() = min_pt.y - 5*_resolution;
    _origin.z() = min_pt.z - 5*_resolution;

    _size.x() = ((max_pt.x+5*_resolution)-_origin.x())*_inverse_resolution;
    _size.y() = ((max_pt.y+5*_resolution)-_origin.y())*_inverse_resolution;
    _size.z() = ((max_pt.z+5*_resolution)-_origin.z())*_inverse_resolution;

    _num_cells = _size.x()*_size.y()*_size.z();

//    size_t lastindex = cloud.find_last_of(".");
//    std::string directory = cloud.substr(0,lastindex);
//    manageDirectories(directory);

    pcl::PCLPointCloud2 out;
    pcl::toPCLPointCloud2(*_cloud,out);
    pcl::io::saveVTKFile("data_set.vtk",out);

}

void BaseGrid::toIdx(float x, float y, float z, Eigen::Vector3i &idx) {
    idx.x() = floor((x - _origin.x())*_inverse_resolution);
    idx.y() = floor((y - _origin.y())*_inverse_resolution);
    idx.z() = floor((z - _origin.z())*_inverse_resolution);
}

void BaseGrid::toWorld(int i, int j, int k, Eigen::Vector3f &point) {
    point.x() = _origin.x() + i*_resolution;
    point.y() = _origin.y() + j*_resolution;
    point.z() = _origin.z() + k*_resolution;
}

int BaseGrid::toInt(Eigen::Vector3i idx) {
    return (idx.z() + idx.y()*_size.z() + idx.x()*_size.y()*_size.z());
}

void BaseGrid::toIJK(int in, Eigen::Vector3i &idx) {
    div_t divresult = div(in,_size.x());
    idx.x() = divresult.rem;
    divresult = div(divresult.quot,_size.y());
    idx.y() = divresult.rem;
    divresult = div(divresult.quot,_size.z());
    idx.z() = divresult.rem;
}

double BaseGrid::computeCloudResolution() {
    double res = 0.0;
    int n_points = 0;

    std::vector<int> indices (2);
    std::vector<float> sqr_distances (2);
    pcl::KdTreeFLANN<PointType> tree;
    tree.setInputCloud (_cloud);

    for (size_t i = 0; i < _cloud->size (); ++i)
    {
        if (! pcl_isfinite ((*_cloud)[i].x))
        {
            continue;
        }
        tree.nearestKSearch (i, 2, indices, sqr_distances);
        res += sqrt (sqr_distances[1]);
        ++n_points;

    }
    if (n_points != 0)
    {
        res /= n_points;
    }
    return res;
}

int BaseGrid::getdigit(double number, int digit) {
    div_t divresult = div(number/pow(10, digit),10);
    return  divresult.rem;
}

void BaseGrid::manageDirectories(std::string directory)
{
    struct stat info;
    if( stat( directory.c_str(), &info ) != 0 )
    {
        mkdir(directory.c_str(),0700);
    }
    else if( info.st_mode & S_IFDIR )
    {
        if(isDirectoryEmpty(directory.c_str()) == 0)
        {
            DIR *theFolder = opendir(directory.c_str());
            struct dirent *next_file;
            char filepath[256];

            while ( (next_file = readdir(theFolder)) != NULL )
            {
                if (0==strcmp(next_file->d_name, ".") || 0==strcmp(next_file->d_name, "..")) { continue; }
                char cwd[1024];
                getcwd(cwd, sizeof(cwd));
                sprintf(filepath, "%s/%s", directory.c_str(), next_file->d_name);
                remove(filepath);
            }
            closedir(theFolder);
        }
    }
    else
        printf( "%s is no directory\n", directory.c_str() );

    chdir(directory.c_str());

}

int BaseGrid::isDirectoryEmpty(const char *dirname)
{
    int n = 0;
    struct dirent *d;
    DIR *dir = opendir(dirname);
    if (dir == NULL)
        return 1;
    while ((d = readdir(dir)) != NULL) {
        if(++n > 2)
            break;
    }
    closedir(dir);
    if (n <= 2)
        return 1;
    else
        return 0;
}


AdaptiveGrid::AdaptiveGrid(const PointCloud::Ptr &cloud, int prec) : BaseGrid(cloud,prec) {

    for(size_t ii=0; ii < _cloud->size(); ii++) {

        float x=_cloud->at(ii).x,y=_cloud->at(ii).y,z=_cloud->at(ii).z;

        Eigen::Vector3i idx;
        toIdx(x,y,z,idx);

        if(hasCell(idx)) {
            Cell* cell = _cells[idx];
            cell->_points.push_back(ii);
            float dist = euclideanDistance(cell->_center,_cloud->at(ii));
            if(dist < cell->_distance) {
                cell->_distance = dist;
                cell->_closest_point = ii;
                cell->_sign = computeSign(cell->_center,_cloud->at(ii));
                //cell->_parent = cell;
            }
        }
        else {
            Cell* cell = new Cell(idx);
            cell->setCenter(_origin,_resolution);
            cell->_points.push_back(ii);
            float dist = euclideanDistance(cell->_center,_cloud->at(ii));
            if(dist < cell->_distance) {
                cell->_distance = dist;
                cell->_closest_point = ii;
                cell->_sign = computeSign(cell->_center,_cloud->at(ii));
            }
            Vector3iCellPtrMap::iterator it = _cells.begin();
            _cells.insert(it,std::pair<Eigen::Vector3i,Cell*>(idx,cell));
        }

    }

    cout << BOLD(FBLU("Building the 3D Grid (Sparse):\n"));
    cout << "\t>> Delta: " << _resolution << "m\n";
    cout << "\t>> Grid dimensions: (" << _size.x() << "," << _size.y() << "," << _size.z() << ")\t total: " << _num_cells << "\n";
    cout << "\t>> Origin: (" << _origin.x() << "," << _origin.y() << "," << _origin.z() << ")\n";
    cout << "\t>> Occupied cells: " << _cells.size() << "\n";
    cout << "--------------------------------------------------------------------------------\n";
    cout << "\n";
}

bool AdaptiveGrid::hasCell(const Eigen::Vector3i &idx) {
    Vector3iCellPtrMap::iterator it = _cells.find(idx);
    return (it != _cells.end()) ? true : false;
}

int AdaptiveGrid::findNeighbors(Cell **neighbors, Cell *c) {
    int x = c->_idx.x();
    int y = c->_idx.y();
    int z = c->_idx.z();
    int xmin = (x-1<0) ? 0 : x-1;
    int xmax = (x+1>_size.x()-1) ? _size.x()-1 : x+1;
    int ymin = (y-1<0) ? 0 : y-1;
    int ymax = (y+1>_size.y()-1) ? _size.y()-1 : y+1;
    int zmin = (z-1<0) ? 0 : z-1;
    int zmax = (z+1>_size.z()-1) ? _size.z()-1 : z+1;
    int k=0;
    for(size_t xx=xmin; xx <= xmax; xx++)
        for(size_t yy=ymin; yy <= ymax; yy++)
            for(size_t zz=zmin; zz <= zmax; zz++)
                if(xx != x || yy != y || zz != z) {
                    Eigen::Vector3i idx(xx,yy,zz);
                    if(hasCell(idx)) {
                        neighbors[k] = _cells[idx];
                        k++;
                    }
                    else {
                        Cell* cell = new Cell(idx);
                        cell->setCenter(_origin,_resolution);
                        Vector3iCellPtrMap::iterator it = _cells.begin();
                        _cells.insert(it,std::pair<Eigen::Vector3i,Cell*>(idx,cell));
                        neighbors[k] = _cells[idx];
                        k++;
                    }
                }
    return k;
}

void AdaptiveGrid::computeDistanceMap(float maxDistance) {
    cout << BOLD(FBLU("Computing Distance Function:\n"));
    cerr << "\t>> Time elapsed: ";
    std::clock_t t0 = clock();
    CellQueue q;
    for(Vector3iCellPtrMap::iterator it = _cells.begin(); it != _cells.end(); ++it) {
        Cell* cell = it->second;
        cell->_parent = cell;
        //cell->_distance = 0;
        q.push(cell);
    }

    bool stop = false;
    int loop = 1;
    Cell* last = 0;
    CellQueue dummy(q);
    while (!dummy.empty()) {
        last = dummy.top();
        dummy.pop();
    }

    Cell* neighbors[26];

    while(stop == false && loop > 0) {
        Cell* current = q.top();
        Cell* parent = current->_parent;
        q.pop();
        int k = findNeighbors(neighbors,current);
        for(int ii=0; ii<k; ii++) {
            Cell* child = neighbors[ii];
            float min_dist = std::numeric_limits<float>::max();
            size_t closest = 0;
            int sign = 0;
            for(size_t jj=0; jj < parent->_points.size(); jj++) {
                size_t index = parent->_points.at(jj);
                float dist = euclideanDistance(child->_center,_cloud->at(index));
                if(dist < min_dist) {
                    min_dist = dist;
                    closest = index;
                    sign = computeSign(child->_center,_cloud->at(index));
                }
            }
            if(min_dist<maxDistance && min_dist<child->_distance) {
                child->_parent = parent;
                child->_distance = min_dist;
                child->_closest_point = closest;
                child->_sign = sign;
                q.push(child);
            }
            //            if(d > 5)
            //                stop = true;

        }
        if(*last == *current) {
            dummy = q;
            while (!dummy.empty()) {
                last = dummy.top();
                dummy.pop();
            }
            loop--;
        }
    }
    std::clock_t t1 = clock();
    double elapsed_time1 = double(t1 - t0)/CLOCKS_PER_SEC;
    cout << elapsed_time1 << "s\n";
    cout << "--------------------------------------------------------------------------------\n";
    cout << "\n";
}

void AdaptiveGrid::writeDataToFile() {
    vtkSmartPointer<vtkImageData> imageData = vtkSmartPointer<vtkImageData>::New();
    imageData->SetDimensions(_size.x(),_size.y(),_size.z());
    imageData->SetOrigin(_origin.x(),_origin.y(),_origin.z());
    imageData->SetSpacing(_resolution,_resolution,_resolution);
    imageData->SetExtent(0,_size.x()-1,0,_size.y()-1,0,_size.z()-1);
#if VTK_MAJOR_VERSION <= 5
    imageData->SetNumberOfScalarComponents(1);
    imageData->SetScalarTypeToFloat();
#else
    imageData->AllocateScalars(VTK_FLOAT, 1);
#endif

    vtkSmartPointer<vtkFloatArray> s_dist = vtkSmartPointer<vtkFloatArray>::New();
    s_dist->SetName("s_distance");
    s_dist->SetNumberOfComponents(1);
    s_dist->SetNumberOfValues(_num_cells);
    vtkSmartPointer<vtkFloatArray> dist = vtkSmartPointer<vtkFloatArray>::New();
    dist->SetName("distance");
    dist->SetNumberOfComponents(1);
    dist->SetNumberOfValues(_num_cells);

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

    for(vtkIdType i=0; i < s_dist->GetNumberOfTuples(); i++) {
        Eigen::Vector3i idx;
        toIJK(i,idx);
        if (hasCell(idx)) {
            dist->SetValue(i,_cells[idx]->_distance);
            s_dist->SetValue(i,_cells[idx]->_sign*_cells[idx]->_distance);
            points->InsertNextPoint(_cells[idx]->_center.x(),_cells[idx]->_center.y(),_cells[idx]->_center.z());
        }
        else {
            dist->SetValue(i,numeric_limits<float>::max());
            s_dist->SetValue(i,numeric_limits<float>::quiet_NaN());
        }
    }
    imageData->GetPointData()->AddArray(s_dist);
    imageData->GetPointData()->AddArray(dist);
    imageData->Update();

    vtkSmartPointer<vtkXMLImageDataWriter> writer = vtkSmartPointer<vtkXMLImageDataWriter>::New();
    writer->SetFileName("solution.vti");
#if VTK_MAJOR_VERSION <= 5
    writer->SetInputConnection(imageData->GetProducerPort());
#else
    writer->SetInputData(imageData);
#endif
    writer->Write();

    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    polydata->SetPoints(points);

    vtkSmartPointer<vtkXMLPolyDataWriter> p_writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
    p_writer->SetFileName("grid.vtp");

#if VTK_MAJOR_VERSION <= 5
    p_writer->SetInput(polydata);
#else
    p_writer->SetInputData(polydata);
#endif
    p_writer->SetDataModeToAscii();
    p_writer->Write();
}
