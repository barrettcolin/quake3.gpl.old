extern "C"
{
#include "tr_local.h"
}

// Generate a plane given a surfType
static void PlaneForSurface(surfaceType_t const *const surfType, cplane_t& plane)
{
    vec4_t plane4 = { 0, 0, 0, 1 };

    if(surfType)
    {
        surfaceType_t const st = *surfType;

        switch(st)
        {
        case SF_FACE:
            plane = ((srfSurfaceFace_t *)surfType)->plane;
            return;
        case SF_TRIANGLES:
            {
                srfTriangles_t const *const tri = (srfTriangles_t *)surfType;
                PlaneFromPoints(plane4, 
                    (tri->verts + tri->indexes[0])->xyz, 
                    (tri->verts + tri->indexes[1])->xyz, 
                    (tri->verts + tri->indexes[2])->xyz);
            }
            break;
        case SF_POLY:
            {
                srfPoly_t const *const poly = (srfPoly_t *)surfType;
                PlaneFromPoints(plane4, 
                    poly->verts[0].xyz, 
                    poly->verts[1].xyz, 
                    poly->verts[2].xyz);
            }
            break;
        }
    }

    VectorCopy(plane4, plane.normal);
    plane.dist = plane4[3];
}

static void MirrorViewParms(cplane_t const& mirrorPlane, orientationr_t const& viewIn, viewParms_t& viewOut)
{
    viewOut.isMirror = qtrue;

    //reflect origin (point)
    float const d = DotProduct(mirrorPlane.normal, viewIn.origin) - mirrorPlane.dist;
    VectorMA(viewIn.origin, -2.0f * d, mirrorPlane.normal, viewOut.or.origin);

    // reflect axis (vectors)
    for(int i = 0; i < 3; i++)
    {
        float const dv = DotProduct(mirrorPlane.normal, viewIn.axis[i]);
        VectorMA(viewIn.axis[i], -2.0f * dv, mirrorPlane.normal, viewOut.or.axis[i]);
    }

    // clip plane
    viewOut.portalPlane = mirrorPlane;
}

static void PortalViewBob(refEntity_t const& ent, vec3_t& camera_axis0, vec3_t& camera_axis1, vec3_t& camera_axis2)
{
    vec3_t transformed;
    if(ent.oldframe)
    {
        if(ent.frame)
        {
            float const d = (tr.refdef.time/1000.0f) * ent.frame;
            VectorCopy(camera_axis1, transformed);
            RotatePointAroundVector(camera_axis1, camera_axis0, transformed, d);
            CrossProduct(camera_axis0, camera_axis1, camera_axis2);
        }
        else
        {
            float const d = ent.skinNum + sin(tr.refdef.time * 0.003f) * 4;
            VectorCopy(camera_axis1, transformed);
            RotatePointAroundVector(camera_axis1, camera_axis0, transformed, d);
            CrossProduct(camera_axis0, camera_axis1, camera_axis2);
        }
    }
    else if(ent.skinNum)
    {
        float const d = ent.skinNum;
        VectorCopy(camera_axis1, transformed);
        RotatePointAroundVector(camera_axis1, camera_axis0, transformed, d);
        CrossProduct(camera_axis0, camera_axis1, camera_axis2);
    }
}

static void PortalViewParms(cplane_t const& portalPlane, orientationr_t const& viewIn, refEntity_t const& ent, viewParms_t& viewOut)
{
    viewOut.isMirror = qfalse;

    // Need to copy these for remote view bobbing
    vec3_t camera_axis0, camera_axis1, camera_axis2;
    {
        VectorCopy(ent.axis[0], camera_axis0);
        VectorCopy(ent.axis[1], camera_axis1);
        VectorCopy(ent.axis[2], camera_axis2);
        PortalViewBob(ent, camera_axis0, camera_axis1, camera_axis2);
    }

    vec3_t const& camera_origin = ent.oldorigin;
    {
        // surface
        vec3_t surface_axis1, surface_axis2, surface_origin;
        {
            PerpendicularVector(surface_axis1, portalPlane.normal);
            CrossProduct(portalPlane.normal, surface_axis1, surface_axis2);

            float const d = DotProduct(ent.origin, portalPlane.normal) - portalPlane.dist;
            VectorMA(ent.origin, -d, portalPlane.normal, surface_origin);
        }

        // view origin
        {
            vec3_t view_offset;
            VectorSubtract(viewIn.origin, surface_origin, view_offset);
            VectorMA(vec3_origin, -DotProduct(view_offset, portalPlane.normal), camera_axis0, viewOut.or.origin);
            VectorMA(viewOut.or.origin, -DotProduct(view_offset, surface_axis1), camera_axis1, viewOut.or.origin);
            VectorMA(viewOut.or.origin, DotProduct(view_offset, surface_axis2), camera_axis2, viewOut.or.origin);
            VectorAdd(viewOut.or.origin, ent.oldorigin, viewOut.or.origin);
        }

        // view axes
        for(int i = 0; i < 3; i++)
        {
            VectorMA(vec3_origin, -DotProduct(viewIn.axis[i], portalPlane.normal), camera_axis0, viewOut.or.axis[i]);
            VectorMA(viewOut.or.axis[i], -DotProduct(viewIn.axis[i], surface_axis1), camera_axis1, viewOut.or.axis[i]);
            VectorMA(viewOut.or.axis[i], DotProduct(viewIn.axis[i], surface_axis2), camera_axis2, viewOut.or.axis[i]);
        }
    }

    // clip plane
    VectorCopy(camera_axis0, viewOut.portalPlane.normal);
    viewOut.portalPlane.dist = DotProduct(camera_origin, viewOut.portalPlane.normal);
}

// Returns qtrue if another view has been rendered
qboolean R_SubviewViewBySurface(drawSurf_t const *const drawSurf, int const entityNum)
{
    // don't recursively mirror
    if(tr.viewParms.isPortal)
    {
        ri.Printf(PRINT_DEVELOPER, "WARNING: recursive mirror/portal found\n");
        return qfalse;
    }

    if(r_noportals->integer || (r_fastsky->integer == 1))
        return qfalse;

    // TODO: portal surfaces part of non-world entities
    cplane_t portalPlane;
    PlaneForSurface(drawSurf->surface, portalPlane);

    viewParms_t newParms = tr.viewParms;
    newParms.isPortal = qtrue;
    for(int i = 0 ; i < tr.refdef.num_entities; i++)
    {
        refEntity_t const& ent = tr.refdef.entities[i].e;

        if(ent.reType != RT_PORTALSURFACE)
            continue;

        float const d = DotProduct(ent.origin, portalPlane.normal) - portalPlane.dist;
        if(d > 64 || d < -64)
            continue;

        VectorCopy(ent.oldorigin, newParms.pvsOrigin);
        if(VectorCompare(ent.oldorigin, ent.origin))
            MirrorViewParms(portalPlane, tr.viewParms.or, newParms);
        else
            PortalViewParms(portalPlane, tr.viewParms.or, ent, newParms);

        viewParms_t oldParms = tr.viewParms;
        R_RenderView(&newParms);
        tr.viewParms = oldParms;
        return qtrue;
    }

    return qfalse;
}
