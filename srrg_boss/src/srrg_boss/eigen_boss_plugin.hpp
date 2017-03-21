
inline void toBOSS(srrg_boss::ObjectData& data, const std::string name) const {
  srrg_boss::ObjectData* matrixData = new srrg_boss::ObjectData;
  if (SizeAtCompileTime == Eigen::Dynamic) {
    matrixData->setInt("rows", rows());
    matrixData->setInt("cols", cols());
  }
  srrg_boss::ArrayData * adata =  new srrg_boss::ArrayData();
  adata->reserve(rows()*cols());
  for (int r=0; r<rows(); r++)
    for (int c=0; c<cols(); c++)
      adata->push_back(new srrg_boss::NumberData((float)this->operator()(r,c)));
  matrixData->setField("values",adata);
  data.setField(name, matrixData);
}

inline void fromBOSS(srrg_boss::ObjectData& data, const std::string name){
  srrg_boss::ObjectData& matrixData = data.getField(name)->getObject();
  if (SizeAtCompileTime == Eigen::Dynamic) {
    int _r = matrixData.getInt("rows");
    int _c = matrixData.getInt("cols");
    if (_r!=0 && _c!=0) 
      this->derived().resize(_r, _c);
    else 
      *this=Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime>();
  }
  srrg_boss::ArrayData& adata = matrixData.getField("values")->getArray();
  assert((int)adata.size()==rows()*cols());
  int k=0;
  for (int r=0; r<rows(); r++)
    for (int c=0; c<cols(); c++, k++)
      this->operator()(r,c) = adata[k];
}


