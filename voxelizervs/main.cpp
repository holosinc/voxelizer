#define VOXELIZER_DEBUG
#define VOXELIZER_IMPLEMENTATION
#include "voxelizer.h"
#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <algorithm>

//#define TRIANGULATE
#define POINT_CLOUD

int main(int argc, char** argv) {
    if (argc != 5) {
        std::cout << "Usage: " << std::endl;
        std::cout << "\t./voxelizer file.obj output.txt maxsize precision" << std::endl;
        return EXIT_FAILURE;
    }

    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string err;
    bool ret = tinyobj::LoadObj(shapes, materials, err, argv[1], NULL);

    if (!err.empty()) {
        std::cerr << err << std::endl;
    }

    if (!ret) {
        return EXIT_FAILURE;
    }

#ifdef TRIANGULATE
    std::ofstream file("mesh_voxelized_res.obj");
#endif

#ifdef POINT_CLOUD
    std::ofstream file(argv[2]);
#endif

    size_t voffset = 0;
    size_t noffset = 0;

	vx_vertex_t min_coords;
	min_coords.x = 0.0f;
	min_coords.y = 0.0f;
	min_coords.z = 0.0f;
	vx_vertex_t max_coords;
	max_coords.x = 0.0f;
	max_coords.y = 0.0f;
	max_coords.z = 0.0f;

	bool firstVertex = true;

	for (size_t i = 0; i < shapes.size(); i++) {
		for (size_t v = 0; v < shapes[i].mesh.positions.size() / 3; v++) {
			float xx = shapes[i].mesh.positions[3 * v + 0];
			float yy = shapes[i].mesh.positions[3 * v + 1];
			float zz = shapes[i].mesh.positions[3 * v + 2];
			if (firstVertex) {
				firstVertex = false;
				min_coords.x = xx;
				min_coords.y = yy;
				min_coords.z = zz;
				max_coords.x = xx;
				max_coords.y = yy;
				max_coords.z = zz;
			} else {
				min_coords.x = std::min(min_coords.x, xx);
				min_coords.y = std::min(min_coords.y, yy);
				min_coords.z = std::min(min_coords.z, zz);
				max_coords.x = std::max(max_coords.x, xx);
				max_coords.y = std::max(max_coords.y, yy);
				max_coords.z = std::max(max_coords.z, zz);
			}
		}
	}

	float dx = max_coords.x - min_coords.x;
	float dy = max_coords.y - min_coords.y;
	float dz = max_coords.z - min_coords.z;

	float dmax = std::max(dx, std::max(dy, dz));
	int maxsize = std::atoi(argv[3]);
	float voxelsize = dmax / maxsize;

	float precision = std::atof(argv[4]);

    for (size_t i = 0; i < shapes.size(); i++) {
        vx_mesh_t* mesh;

        mesh = vx_mesh_alloc(shapes[i].mesh.positions.size(), shapes[i].mesh.indices.size());

        for (size_t f = 0; f < shapes[i].mesh.indices.size(); f++) {
            mesh->indices[f] = shapes[i].mesh.indices[f];
        }
        for (size_t v = 0; v < shapes[i].mesh.positions.size() / 3; v++) {
            mesh->vertices[v].x = shapes[i].mesh.positions[3*v+0];
            mesh->vertices[v].y = shapes[i].mesh.positions[3*v+1];
            mesh->vertices[v].z = shapes[i].mesh.positions[3*v+2];
        }

#ifdef TRIANGULATE
        vx_mesh_t* result;
        result = vx_voxelize(mesh, voxelsize, voxelsize, voxelsize, precision);

        printf("Number of vertices: %ld\n", result->nvertices);
        printf("Number of indices: %ld\n", result->nindices);

        if (file.is_open()) {
            file << "\n";
            file << "o " << i << "\n";

            for (int j = 0; j < result->nvertices; ++j) {
                file << "v " << result->vertices[j].x << " "
                             << result->vertices[j].y << " "
                             << result->vertices[j].z << "\n";
            }

            for (int j = 0; j < result->nnormals; ++j) {
                file << "vn " << result->normals[j].x << " "
                              << result->normals[j].y << " "
                              << result->normals[j].z << "\n";
            }

            size_t max = 0;

            for (int j = 0; j < result->nindices; j += 3) {
                size_t i0 = voffset + result->indices[j+0] + 1;
                size_t i1 = voffset + result->indices[j+1] + 1;
                size_t i2 = voffset + result->indices[j+2] + 1;

                max = i0 > max ? i0 : max;
                max = i1 > max ? i1 : max;
                max = i2 > max ? i2 : max;

                file << "f ";

                file << i0 << "//" << result->normalindices[j+0] + noffset + 1 << " ";
                file << i1 << "//" << result->normalindices[j+1] + noffset + 1 << " ";
                file << i2 << "//" << result->normalindices[j+2] + noffset + 1 << "\n";
            }

            voffset += max;
            noffset += 6;
        }

        vx_mesh_free(result);
        vx_mesh_free(mesh);
#endif

#ifdef POINT_CLOUD
        vx_point_cloud_t* result;
        result = vx_voxelize_pc(mesh, voxelsize, voxelsize, voxelsize, precision);

        printf("Number of vertices: %ld\n", result->nvertices);

        if (file.is_open()) {
            file << "\n";
            for (int j = 0; j < result->nvertices; ++j) {
                file << result->vertices[j].x << " "
                     << result->vertices[j].y << " "
                     << result->vertices[j].z << "\n";
            }
        }

        vx_point_cloud_free(result);
        vx_mesh_free(mesh);
#endif
    }

    file.close();

    return EXIT_SUCCESS;
}
