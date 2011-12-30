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

static void PortalViewParms(cplane_t const& portalPlane, 
                            orientationr_t const& viewIn, 
                            refEntity_t const& misc_portal_surface, 
                            vec3_t const& misc_portal_surface_world_origin, 
                            viewParms_t& viewOut)
{
    viewOut.isMirror = qfalse;

    // Need to copy these for remote view bobbing
    vec3_t camera_axis0, camera_axis1, camera_axis2;
    {
        VectorCopy(misc_portal_surface.axis[0], camera_axis0);
        VectorCopy(misc_portal_surface.axis[1], camera_axis1);
        VectorCopy(misc_portal_surface.axis[2], camera_axis2);
        PortalViewBob(misc_portal_surface, camera_axis0, camera_axis1, camera_axis2);
    }

    vec3_t const& camera_origin = misc_portal_surface.oldorigin;
    {
        // surface
        vec3_t surface_axis1, surface_axis2, surface_origin;
        {
            PerpendicularVector(surface_axis1, portalPlane.normal);
            CrossProduct(portalPlane.normal, surface_axis1, surface_axis2);

            float const d = DotProduct(misc_portal_surface_world_origin, portalPlane.normal) - portalPlane.dist;
            VectorMA(misc_portal_surface_world_origin, -d, portalPlane.normal, surface_origin);
        }

        // view origin
        {
            vec3_t view_offset;
            VectorSubtract(viewIn.origin, surface_origin, view_offset);
            VectorMA(vec3_origin, -DotProduct(view_offset, portalPlane.normal), camera_axis0, viewOut.or.origin);
            VectorMA(viewOut.or.origin, -DotProduct(view_offset, surface_axis1), camera_axis1, viewOut.or.origin);
            VectorMA(viewOut.or.origin, DotProduct(view_offset, surface_axis2), camera_axis2, viewOut.or.origin);
            VectorAdd(viewOut.or.origin, misc_portal_surface.oldorigin, viewOut.or.origin);
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
qboolean R_SubviewViewBySurface(drawSurf_t const *const drawSurf, int const drawSurfEntityNum)
{
    // Limit mirror recursions
    if(tr.viewParms.isPortal && tr.viewParms.subview_level >= r_subviewRecurse->integer)
        return qfalse;

    if(r_noportals->integer || (r_fastsky->integer == 1))
        return qfalse;

    // Add stencil surface
    R_AddStencilSurfCmd(drawSurf);

    // Portal plane
    cplane_t originalPlane, portalPlane;
    PlaneForSurface(drawSurf->surface, originalPlane);

    bool const draw_surf_entity_is_world = (ENTITYNUM_WORLD == drawSurfEntityNum);
    if(!draw_surf_entity_is_world)
    {
        // write portal surface entity orientation to tr.or
        R_RotateForEntity(&tr.refdef.entities[drawSurfEntityNum], &tr.viewParms, &tr.or);
        // transform by tr.or
        R_LocalNormalToWorld(originalPlane.normal, portalPlane.normal);
        // portal plane is relative to entity origin; move to world
        portalPlane.dist = DotProduct(portalPlane.normal, tr.or.origin) + originalPlane.dist;
        // original plane is relative to entity origin; move to world
        originalPlane.dist += DotProduct(originalPlane.normal, tr.or.origin);
    }
    else
        portalPlane = originalPlane;

    viewParms_t new_parms = tr.viewParms;
    new_parms.isPortal = qtrue;
    for(int i = 0 ; i < tr.refdef.num_entities; i++)
    {
        refEntity_t const& misc_portal_surface = tr.refdef.entities[i].e;

        if(misc_portal_surface.reType != RT_PORTALSURFACE)
            continue;

        float const d = DotProduct(misc_portal_surface.origin, originalPlane.normal) - originalPlane.dist;
        if(d > 64 || d < -64)
            continue;

        vec3_t misc_portal_surface_world;
        if(!draw_surf_entity_is_world)
        {
            vec3_t misc_portal_surface_local;
            // misc_portal_surface_local = misc_portal_surface.origin - tr.or.origin
            VectorSubtract(misc_portal_surface.origin, tr.or.origin, misc_portal_surface_local);
            // misc_portal_surface_world = misc_portal_surface_local * tr.or
            R_LocalPointToWorld(misc_portal_surface_local, misc_portal_surface_world);
        }
        else
            VectorCopy(misc_portal_surface.origin, misc_portal_surface_world);

        VectorCopy(misc_portal_surface.oldorigin, new_parms.pvsOrigin);
        if(VectorCompare(misc_portal_surface.oldorigin, misc_portal_surface.origin))
            MirrorViewParms(portalPlane, tr.viewParms.or, new_parms);
        else
            PortalViewParms(portalPlane, tr.viewParms.or, misc_portal_surface, misc_portal_surface_world, new_parms);

        viewParms_t old_parms = tr.viewParms;
        new_parms.subview_level = old_parms.subview_level + 1;
        R_RenderView(&new_parms);
        tr.viewParms = old_parms;
        return qtrue;
    }

    return qfalse;
}
