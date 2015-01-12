/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
#include "flyvr/StimulusInterface.hpp"
#include "flyvr/flyvr_assert.h"

#include "json2osg.hpp"

#include "Poco/ClassLibrary.h"

#include <iostream>
#include <stdio.h>
#include <string.h>

#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/Geode>

#include <OpenThreads/ScopedLock>

#define NUM_GRATINGS 1

#include <math.h>
const static double D2R = M_PI/180.0;

typedef struct
{
    bool  reset_phase_position;
    float phase_position;
    float phase_velocity;
    float wavelength;
    float contrast;
    float orientation;
} GratingParams;

GratingParams parse_grating_info(const json_t * const root) {
    GratingParams result;

    json_t *data_json;

    data_json = json_object_get(root, "reset_phase_position");
    flyvr_assert(data_json != NULL);
    flyvr_assert(json_is_boolean(data_json));
    result.reset_phase_position = json_is_true( data_json );

    data_json = json_object_get(root, "phase_position");
    flyvr_assert(data_json != NULL);
    flyvr_assert(json_is_real(data_json));
    result.phase_position = json_real_value( data_json );

    data_json = json_object_get(root, "phase_velocity");
    flyvr_assert(data_json != NULL);
    flyvr_assert(json_is_real(data_json));
    result.phase_velocity = json_real_value( data_json );

    data_json = json_object_get(root, "wavelength");
    flyvr_assert(data_json != NULL);
    flyvr_assert(json_is_real(data_json));
    result.wavelength = json_real_value( data_json );

    data_json = json_object_get(root, "contrast");
    flyvr_assert(data_json != NULL);
    flyvr_assert(json_is_real(data_json));
    result.contrast = json_real_value( data_json );

    data_json = json_object_get(root, "orientation");
    flyvr_assert(data_json != NULL);
    flyvr_assert(json_is_real(data_json));
    result.orientation = json_real_value( data_json );

    return result;
}

typedef struct
{
    OpenThreads::Mutex                  mutex;
    bool                                dirty;
    GratingParams                       params;
    osg::ref_ptr<osg::Uniform>          u_phase_position;
    osg::ref_ptr<osg::Uniform>          u_wavelength;
    osg::ref_ptr<osg::Uniform>          u_contrast;
    osg::ref_ptr<osg::Uniform>          u_orientation;
} GratingType;

typedef struct
{
    osg::ref_ptr<osg::Cylinder>         cylinder;
    osg::ref_ptr<osg::ShapeDrawable>    shape;
    osg::ref_ptr<osg::Geode>            geode;

    osg::ref_ptr<osg::StateSet>         state_set;

    osg::ref_ptr<osg::Program>          program;
    osg::ref_ptr<osg::Shader>           vertex_shader;
    osg::ref_ptr<osg::Shader>           fragment_shader;

    GratingType gratings[NUM_GRATINGS];
} CylInfo;

class StimulusCylinderGrating_centered: public StimulusInterface
{
public:
    StimulusCylinderGrating_centered();

    std::vector<std::string> get_topic_names() const;
    void receive_json_message(const std::string& topic_name, const std::string& json_message);
    std::string get_message_type(const std::string& topic_name) const;
    void update( const double& time, const osg::Vec3& observer_position, const osg::Quat& observer_orientation );

    std::string name() const { return "StimulusCylinderGrating_centered"; }
    osg::ref_ptr<osg::Group> get_3d_world() {return _virtual_world; }

private:
    osg::ref_ptr<osg::Group>            _virtual_world;
    double                              _t0;
    CylInfo _cyl;

    osg::ref_ptr<osg::Group> create_virtual_world();
    void post_init(bool);
    void init_cyl(CylInfo&);
    void set_grating_info( int i, GratingParams& new_values);
};

StimulusCylinderGrating_centered::StimulusCylinderGrating_centered() :
    _t0(-1)
{
    _virtual_world = create_virtual_world();
}

std::vector<std::string> StimulusCylinderGrating_centered::get_topic_names() const
{
    std::vector<std::string> result;
	result.push_back("grating_info");
    return result;
}

void StimulusCylinderGrating_centered::receive_json_message(const std::string& topic_name, const std::string& json_message)
{
    json_t *root;
    json_error_t error;

    root = json_loads(json_message.c_str(), 0, &error);
    flyvr_assert(root != NULL);

    if (topic_name=="grating_info") {
        GratingParams new_values;
        json_t *data_json;

        //data_json = json_object_get(root, "grating0");
        //new_values = parse_grating_info(data_json);
        new_values = parse_grating_info(root);
        set_grating_info(0,new_values);

        //data_json = json_object_get(root, "grating1");
        //new_values = parse_grating_info(data_json);
        //set_grating_info(1,new_values);
    } else {
        throw std::runtime_error("unknown topic name");
    }

    json_decref(root);
}

std::string StimulusCylinderGrating_centered::get_message_type(const std::string& topic_name) const
{
    std::string result;
    if (topic_name=="grating_info") {
        result = "flyvr/CylinderGratingInfo";
    } else {
        throw std::runtime_error("unknown topic name");
    }
    return result;
}

void StimulusCylinderGrating_centered::update( const double& time, const osg::Vec3& observer_position, const osg::Quat& observer_orientation )
{
    double dt=0.0;
    if (_t0 > 0) {
        // update by dt seconds
        dt = time-_t0;
        _t0 = time;
    } else {
        // first iteration
        _t0 = time;
    }

    for (int i=0; i<NUM_GRATINGS; i++) {
        GratingType &grating = _cyl.gratings[i];
        {
            OpenThreads::ScopedLock<OpenThreads::Mutex> lock(grating.mutex);

            if (grating.dirty) {
                grating.dirty = false; // reset dirty flag

                grating.u_phase_position->set(grating.params.phase_position);
                grating.u_wavelength->set(grating.params.wavelength);
                grating.u_contrast->set(grating.params.contrast);
                grating.u_orientation->set(grating.params.orientation);
            }

            if (grating.params.phase_velocity != 0) {
                grating.params.phase_position += dt*grating.params.phase_velocity;
                grating.u_phase_position->set(grating.params.phase_position);
            }
        }

    }
}

void StimulusCylinderGrating_centered::init_cyl(CylInfo& cyl) {

    osg::ref_ptr<osg::TessellationHints> hints = new osg::TessellationHints;
    hints->setDetailRatio(2.0f);
    hints->setCreateTop(false);
    hints->setCreateBottom(false);

    osg::Vec3 center = osg::Vec3(0.0f,0.0f,0.0f);
    float radius = 1.0f;
    float height = 3.0;

    cyl.cylinder = new osg::Cylinder(center,radius,height);
    cyl.shape = new osg::ShapeDrawable(cyl.cylinder, hints.get());

    const char* phase_position_name;
    const char* wavelength_name;
    const char* contrast_name;
    const char* orientation_name;

    for (int i=0; i<NUM_GRATINGS; i++) {
        GratingParams new_values;
        new_values.phase_position = 0.0;
        new_values.phase_velocity = 360*D2R;
        new_values.wavelength = 20*D2R;
        new_values.contrast = 1.0;
      new_values.orientation = 0;

        if (i==0) {
            phase_position_name="phase_position0";
            wavelength_name="wavelength0";
            contrast_name="contrast0";
            orientation_name="orientation0";
        } else {
            flyvr_assert(false);
        }

        cyl.gratings[i].u_phase_position = new osg::Uniform( osg::Uniform::FLOAT, phase_position_name );
        cyl.gratings[i].u_wavelength = new osg::Uniform( osg::Uniform::FLOAT, wavelength_name );
        cyl.gratings[i].u_contrast = new osg::Uniform( osg::Uniform::FLOAT, contrast_name );
        cyl.gratings[i].u_orientation = new osg::Uniform( osg::Uniform::FLOAT, orientation_name );

        set_grating_info( i, new_values);
    }

    cyl.geode = new osg::Geode;
    cyl.geode->addDrawable(cyl.shape.get());

    cyl.program = new osg::Program;
    cyl.program->setName( "cylinder_shader" );

    cyl.vertex_shader = new osg::Shader( osg::Shader::VERTEX );
    cyl.fragment_shader = new osg::Shader( osg::Shader::FRAGMENT );

    cyl.program->addShader(cyl.vertex_shader);
    cyl.program->addShader(cyl.fragment_shader);

    load_shader_source( cyl.vertex_shader, "grating_blended_centered.vert" );
    load_shader_source( cyl.fragment_shader, "grating_blended_centered_ed.frag" );

    cyl.state_set = cyl.shape->getOrCreateStateSet();

    cyl.state_set->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    cyl.state_set->setMode(GL_BLEND, osg::StateAttribute::ON);

    cyl.state_set->setAttributeAndModes( cyl.program, osg::StateAttribute::ON);

    for (int i=0;i<NUM_GRATINGS;i++) {
        cyl.state_set->addUniform( cyl.gratings[i].u_phase_position );
        cyl.state_set->addUniform( cyl.gratings[i].u_wavelength );
        cyl.state_set->addUniform( cyl.gratings[i].u_contrast );
        cyl.state_set->addUniform( cyl.gratings[i].u_orientation );
    }

}

osg::ref_ptr<osg::Group> StimulusCylinderGrating_centered::create_virtual_world() {
    osg::ref_ptr<osg::MatrixTransform> myroot = new osg::MatrixTransform; myroot->addDescription("virtual world root node");


    return myroot;
}

void StimulusCylinderGrating_centered::post_init(bool slave)
{

    init_cyl(_cyl);

    _virtual_world->addChild( _cyl.geode.get() );
}

void StimulusCylinderGrating_centered::set_grating_info(int i, GratingParams &new_values) {
    GratingType &grating = _cyl.gratings[i];
    {
        OpenThreads::ScopedLock<OpenThreads::Mutex> lock(grating.mutex);

        float orig_position = grating.params.phase_position;
        grating.params = new_values;
        if (!new_values.reset_phase_position) {
            grating.params.phase_position = orig_position;
        }
        grating.dirty = true;
    }
}

POCO_BEGIN_MANIFEST(StimulusInterface)
POCO_EXPORT_CLASS(StimulusCylinderGrating_centered)
POCO_END_MANIFEST

void pocoInitializeLibrary()
{
}

void pocoUninitializeLibrary()
{
}
