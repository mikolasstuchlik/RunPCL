#include "include/CppPCL.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/io/ply_io.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include <pcl/io/obj_io.h>

#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>

// https://pcl.readthedocs.io/projects/tutorials/en/master/bspline_fitting.html#bspline-fitting
void
PointCloud2Vector3d (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data);
int
fittingSurface (std::string path);

int publicFittingSurface(const char * _Nonnull path) {
    std::string stdPath = path;
    return fittingSurface(stdPath);
}

int
fittingSurface (std::string path)
{
//    std::string pcd_file, file_3dm;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PLYReader Reader;
    Reader.read(path, *cloud);


    // ############################################################################
    // load point cloud

//    printf ("  loading %s\n", pcd_file.c_str ());

//    pcl::PCLPointCloud2 cloud2;
    pcl::on_nurbs::NurbsDataSurface data;
//
//    if (pcl::io::loadPCDFile (pcd_file, cloud2) == -1)
//        throw std::runtime_error ("  PCD file not found.");

//    fromPCLPointCloud2 (cloud2, *cloud);
    PointCloud2Vector3d (cloud, data.interior);
    printf ("  %lu points in data set\n", cloud->size ());

    // ############################################################################
    // fit B-spline surface

    // parameters
    unsigned order (3);
    unsigned refinement (5);
    unsigned iterations (10);
    unsigned mesh_resolution (256);

    pcl::on_nurbs::FittingSurface::Parameter params;
    params.interior_smoothness = 0.2;
    params.interior_weight = 1.0;
    params.boundary_smoothness = 0.2;
    params.boundary_weight = 0.0;

    // initialize
    printf ("  surface fitting ...\n");
    ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox (order, &data);
    pcl::on_nurbs::FittingSurface fit (&data, nurbs);
    //  fit.setQuiet (false); // enable/disable debug output

    
    // mesh for visualization
    pcl::PolygonMesh mesh;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::Vertices> mesh_vertices;
    std::string mesh_id = "mesh_nurbs";
    pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh (fit.m_nurbs, mesh, mesh_resolution);

    // surface refinement
    for (unsigned i = 0; i < refinement; i++)
    {
        fit.refine (0);
        fit.refine (1);
        fit.assemble (params);
        fit.solve ();
        pcl::on_nurbs::Triangulation::convertSurface2Vertices (fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
        printf ("  refine...\n");
    }

    // surface fitting with final refinement level
    for (unsigned i = 0; i < iterations; i++)
    {
        fit.assemble (params);
        fit.solve ();
        pcl::on_nurbs::Triangulation::convertSurface2Vertices (fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
        printf ("  assemble ...\n");
    }

    // ############################################################################
    // fit B-spline curve

    // parameters
    pcl::on_nurbs::FittingCurve2dAPDM::FitParameter curve_params;
    curve_params.addCPsAccuracy = 5e-2;
    curve_params.addCPsIteration = 3;
    curve_params.maxCPs = 200;
    curve_params.accuracy = 1e-3;
    curve_params.iterations = 100;

    curve_params.param.closest_point_resolution = 0;
    curve_params.param.closest_point_weight = 1.0;
    curve_params.param.closest_point_sigma2 = 0.1;
    curve_params.param.interior_sigma2 = 0.00001;
    curve_params.param.smooth_concavity = 1.0;
    curve_params.param.smoothness = 1.0;

    // initialisation (circular)
    printf ("  curve fitting ...\n");
    pcl::on_nurbs::NurbsDataCurve2d curve_data;
    curve_data.interior = data.interior_param;
    curve_data.interior_weight_function.push_back (true);
    ON_NurbsCurve curve_nurbs = pcl::on_nurbs::FittingCurve2dAPDM::initNurbsCurve2D (order, curve_data.interior);

    // curve fitting
    pcl::on_nurbs::FittingCurve2dASDM curve_fit (&curve_data, curve_nurbs);
    // curve_fit.setQuiet (false); // enable/disable debug output
    curve_fit.fitting (curve_params);

    // ############################################################################
    // triangulation of trimmed surface

    printf ("  triangulate trimmed surface ...\n");
    pcl::on_nurbs::Triangulation::convertTrimmedSurface2PolygonMesh (fit.m_nurbs, curve_fit.m_nurbs, mesh,
                                                                     mesh_resolution);


    // save trimmed B-spline surface
//    if ( fit.m_nurbs.IsValid() )
//    {
//        ONX_Model model;
//        ONX_Model_Object& surf = model.m_object_table.AppendNew();
//        surf.m_object = new ON_NurbsSurface(fit.m_nurbs);
//        surf.m_bDeleteObject = true;
//        surf.m_attributes.m_layer_index = 1;
//        surf.m_attributes.m_name = "surface";
//
//        ONX_Model_Object& curv = model.m_object_table.AppendNew();
//        curv.m_object = new ON_NurbsCurve(curve_fit.m_nurbs);
//        curv.m_bDeleteObject = true;
//        curv.m_attributes.m_layer_index = 2;
//        curv.m_attributes.m_name = "trimming curve";
//
//        model.Write(file_3dm.c_str());
//        printf("  model saved: %s\n", file_3dm.c_str());
//    }

    pcl::io::saveOBJFile("pcl_saveOBJFile2.obj", mesh);

    printf ("  ... done.\n");

    return 0;
}


void
PointCloud2Vector3d (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data)
{
    for (unsigned i = 0; i < cloud->size (); i++)
    {
        pcl::PointXYZ &p = cloud->at (i);
        if (!std::isnan (p.x) && !std::isnan (p.y) && !std::isnan (p.z))
            data.push_back (Eigen::Vector3d (p.x, p.y, p.z));
    }
}

