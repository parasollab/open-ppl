#ifndef BVH_DATA_LOADER_H_
#define BVH_DATA_LOADER_H_

#include <fstream>
#include <string>
#include <vector>

#include "IModel.h"


////////////////////////////////////////////////////////////////////////////////
/// \brief A BVH data loader. The data is broken up into edges per keyframe.
////////////////////////////////////////////////////////////////////////////////
class CBVHDataLoader : public IModel {

  public:

    void load(std::ifstream& _in);

    ///\name IModel Overrides
    ///@{

    virtual bool ParseFile(bool) override;

    ///@}
};


////////////////////////////////////////////////////////////////////////////////
/// \brief ?
////////////////////////////////////////////////////////////////////////////////
class CBVHDataInstance {

  public:

    ///\name Construction
    ///@{

    CBVHDataInstance() : m_id(-1) { }

    ///@}
    ///\name Accessors
    ///@{

    std::string getFileName() {return m_filename;}
    void setFileName(const std::string& _fn) {m_filename = _fn;}

    void setInstanceID(int _id) {m_id = _id;}
    int getInstanceID() {return m_id;}

    std::vector<IModel*>& getModels() {return m_models;}

    ///@}
    ///\name Loading Function
    ///@{

    void load(std::string file, double radius, double height);

    ///@}
    ///\name Internal State
    ///@{

    std::string m_filename;
    std::vector<IModel*> m_models;
    int m_id;

    ///@}
};


////////////////////////////////////////////////////////////////////////////////
/// \brief ?
////////////////////////////////////////////////////////////////////////////////
class CBVHDataModelFactory {

  public:

    ///\name Instance Accessors
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Get the number of instances.
    size_t numInstances();

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Add an instance to the factory's instance set.
    void addInstance(CBVHDataInstance* _i);

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Get an instance from the factory's instance set.
    CBVHDataInstance* getInstance(size_t _index);

    ////////////////////////////////////////////////////////////////////////////
    /// \brief
    bool hasBVHDataInstance(const std::string& _filename, int& _index);

    ///@}
    ///\name Internal State
    ///@{

    std::vector<CBVHDataInstance*> m_BVHDataInstances;

    ///@}
};

#endif
