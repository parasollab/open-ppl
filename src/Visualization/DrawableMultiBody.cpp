#include "DrawableMultiBody.h"

#include "Geometry/Bodies/ActiveMultiBody.h"
#include "Geometry/Bodies/FixedBody.h"
#include "Geometry/Bodies/FreeBody.h"
#include "Geometry/Bodies/MultiBody.h"
#include "Geometry/Bodies/StaticMultiBody.h"


/*------------------------------- Construction -------------------------------*/

DrawableMultiBody::
DrawableMultiBody(MultiBody* _m) {
  switch(_m->GetType()) {
    case MultiBody::MultiBodyType::Active:
      {
        ActiveMultiBody* aMB = dynamic_cast<ActiveMultiBody*>(_m);
        for(size_t i = 0; i < aMB->GetNumBodies(); ++i)
          m_bodies.emplace_back(this, aMB->GetFreeBody(i));
      }
      break;
    case MultiBody::MultiBodyType::Passive:
      {
        StaticMultiBody* sMB = dynamic_cast<StaticMultiBody*>(_m);
        for(size_t i = 0; i < sMB->GetNumBodies(); ++i)
          m_bodies.emplace_back(this, sMB->GetFixedBody(i));
      }
      break;
    case MultiBody::MultiBodyType::Internal:
      break;
    default:
      throw RunTimeException(WHERE, "Unrecognized MultiBody type");
  }
}

/*---------------------------- MultiBody Support -----------------------------*/

size_t
DrawableMultiBody::
GetNumBodies() const noexcept {
  return m_bodies.size();
}


void
DrawableMultiBody::
PushTransform(const size_t _i, const glutils::transform& _t) {
  m_bodies[_i].push_transform(_t);
}


void
DrawableMultiBody::
UpdateTransform() {
  for(auto& b : m_bodies)
    b.update_transform();
}

/*--------------------------- Drawable Overrides -----------------------------*/

void
DrawableMultiBody::
draw() {
  for(auto& b : m_bodies)
    b.render();
}


void
DrawableMultiBody::
draw_select() {
  for(auto& b : m_bodies)
    b.render_select();
}


void
DrawableMultiBody::
draw_selected() {
  /// @TODO
}


void
DrawableMultiBody::
draw_highlighted() {
  /// @TODO
}


void
DrawableMultiBody::
select() noexcept {
  for(auto& body : m_bodies)
    body.glutils::drawable::select();
}


void
DrawableMultiBody::
deselect() noexcept {
  for(auto& body : m_bodies)
    body.glutils::drawable::deselect();
}


void
DrawableMultiBody::
highlight() noexcept {
  for(auto& body : m_bodies)
    body.glutils::drawable::highlight();
}


void
DrawableMultiBody::
unhighlight() noexcept {
  for(auto& body : m_bodies)
    body.glutils::drawable::unhighlight();
}

/*----------------------------------------------------------------------------*/
